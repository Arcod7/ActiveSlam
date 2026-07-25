// Rebuilds an octomap::OcTree from the VDBFusion TSDF grid.
//
// tsdf_mapper owns the belief map but exposes it as clouds, not as an octree:
//   /tsdf/occupied_voxels  confidently solid voxel centres (d <= voxel_max_d)
//   /tsdf/free_voxels      observed-empty voxel centres    (d > 0)
// Everything absent from both stays unknown, which is exactly the
// free/occupied/unknown split 3-D frontier detection and 3-D A* need.
//
// The tree is rebuilt from scratch each cycle rather than accumulated: the
// TSDF is the single source of truth and is itself reset+re-integrated after a
// large loop closure (map_rebuild:=true), so an incrementally-updated octree
// would keep stale cells the TSDF has already corrected away.

#include <memory>
#include <string>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;

std::vector<octomap::point3d> toPoints(const PointCloud2 & msg)
{
  std::vector<octomap::point3d> pts;
  pts.reserve(msg.width * msg.height);
  sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
  for (; ix != ix.end(); ++ix, ++iy, ++iz) {
    if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
      continue;
    }
    pts.emplace_back(*ix, *iy, *iz);
  }
  return pts;
}
}  // namespace

class TsdfToOctomap : public rclcpp::Node
{
public:
  TsdfToOctomap()
  : Node("tsdf_to_octomap")
  {
    // Must match tsdf_mapper's voxel_size, or cells land off-grid.
    resolution_ = declare_parameter<double>("resolution", 0.2);
    world_frame_ = declare_parameter<std::string>("world_frame", "world_ned");
    const auto occupied_topic =
      declare_parameter<std::string>("occupied_topic", "/tsdf/occupied_voxels");
    const auto free_topic =
      declare_parameter<std::string>("free_topic", "/tsdf/free_voxels");
    const double period_s = declare_parameter<double>("publish_period_s", 1.0);
    // Set directly rather than raycast-accumulated: one observation of a TSDF
    // cell already carries the whole integration history behind it.
    occupied_logodds_ = octomap::logodds(declare_parameter<double>("occupied_prob", 0.9));
    free_logodds_ = octomap::logodds(declare_parameter<double>("free_prob", 0.1));
    publish_full_ = declare_parameter<bool>("publish_full", false);

    auto qos = rclcpp::QoS(1);
    occupied_sub_ = create_subscription<PointCloud2>(
      occupied_topic, qos, [this](PointCloud2::ConstSharedPtr msg) {
        occupied_ = toPoints(*msg);
        dirty_ = true;
      });
    free_sub_ = create_subscription<PointCloud2>(
      free_topic, qos, [this](PointCloud2::ConstSharedPtr msg) {
        free_ = toPoints(*msg);
        dirty_ = true;
      });

    // Latched: RViz and any planner that starts late still get the map.
    auto pub_qos = rclcpp::QoS(1).transient_local();
    binary_pub_ = create_publisher<octomap_msgs::msg::Octomap>("/octomap_binary", pub_qos);
    if (publish_full_) {
      full_pub_ = create_publisher<octomap_msgs::msg::Octomap>("/octomap_full", pub_qos);
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_s), [this]() {rebuild();});

    RCLCPP_INFO(
      get_logger(),
      "tsdf_to_octomap ready — %s + %s -> /octomap_binary at %.2f m, every %.1fs",
      occupied_topic.c_str(), free_topic.c_str(), resolution_, period_s);
  }

private:
  void rebuild()
  {
    if (!dirty_) {
      return;
    }
    dirty_ = false;

    octomap::OcTree tree(resolution_);
    // Free first so an occupied cell wins any coordinate claimed by both.
    for (const auto & p : free_) {
      tree.setNodeValue(p, free_logodds_, true);
    }
    for (const auto & p : occupied_) {
      tree.setNodeValue(p, occupied_logodds_, true);
    }
    tree.updateInnerOccupancy();
    tree.prune();

    octomap_msgs::msg::Octomap msg;
    msg.header.frame_id = world_frame_;
    msg.header.stamp = now();
    if (octomap_msgs::binaryMapToMsg(tree, msg)) {
      binary_pub_->publish(msg);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "binaryMapToMsg failed; no octomap published");
      return;
    }
    if (full_pub_) {
      octomap_msgs::msg::Octomap full;
      full.header = msg.header;
      if (octomap_msgs::fullMapToMsg(tree, full)) {
        full_pub_->publish(full);
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "OcTree: %zu occupied + %zu free cells -> %zu nodes",
      occupied_.size(), free_.size(), tree.size());
  }

  double resolution_;
  std::string world_frame_;
  float occupied_logodds_;
  float free_logodds_;
  bool publish_full_{false};
  bool dirty_{false};
  std::vector<octomap::point3d> occupied_;
  std::vector<octomap::point3d> free_;
  rclcpp::Subscription<PointCloud2>::SharedPtr occupied_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr free_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr binary_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr full_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TsdfToOctomap>());
  rclcpp::shutdown();
  return 0;
}
