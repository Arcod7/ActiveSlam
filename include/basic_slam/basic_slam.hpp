#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>


class basic_slam : public rclcpp::Node
{
public:
    basic_slam();


private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr process_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);


    void reset_topic_check();
    bool topic_check[4] = {false, false, false, false};
    sensor_msgs::msg::PointCloud2::SharedPtr depthcam_pointcloud = nullptr;
    sensor_msgs::msg::Image::SharedPtr depth_cam_image = nullptr;
    sensor_msgs::msg::Image::SharedPtr segmentation_image = nullptr;
    nav_msgs::msg::Odometry::SharedPtr odometry = nullptr;

    double _sensor_position[3];
    double _sensor_rotation_quaternion[4];
    void get_odom_from_ros();



    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cam_pt_subscription_;
    void depth_cam_pt_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_cam_image_subscription_;
    void depth_cam_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr segmentation_cam_subscription_;
    void segmentation_cam_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void process_imput();
};
