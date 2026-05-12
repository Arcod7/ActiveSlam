#include "basic_slam/basic_slam.hpp"

using std::placeholders::_1;

basic_slam::basic_slam() : Node("processing_node")
{
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sensor_msgs/processed_pointcloud", 10);

  depth_cam_pt_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensor_msgs/pointcloud", 10, std::bind(&basic_slam::depth_cam_pt_callback, this, _1));

  depth_cam_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/sensor_msgs/pointcloud", 10, std::bind(&basic_slam::depth_cam_image_callback, this, _1));

  segmentation_cam_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/StoneFish/Segmentation/image_color", 10, std::bind(&basic_slam::segmentation_cam_callback, this, _1));
  
  odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/StoneFish/Odometry", 10, std::bind(&basic_slam::odometry_callback, this, _1));

}

void basic_slam::reset_topic_check(){
  for (bool &check : topic_check) {
    check = false;
  }
  depthcam_pointcloud = nullptr;
  depth_cam_image = nullptr;
  segmentation_image = nullptr;
  odometry = nullptr;
  process_imput();
}

void basic_slam::depth_cam_pt_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  depthcam_pointcloud = msg;
  topic_check[0] = true;
  process_imput();
  std::cout << "Pointcloud received !" << std::endl;


}
void basic_slam::depth_cam_image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
  depth_cam_image = msg;
  topic_check[1] = true;
  process_imput();
  std::cout << "Image received !" << std::endl;

}
void basic_slam::segmentation_cam_callback(const sensor_msgs::msg::Image::SharedPtr msg){
  segmentation_image = msg;
  topic_check[2] = true;
  process_imput();
  std::cout << "Segmentation received !" << std::endl;

}
void basic_slam::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  odometry = msg;
  topic_check[3] = true;
  process_imput();
  std::cout << "Odometry received !" << std::endl;


}




pcl::PointCloud<pcl::PointXYZ>::Ptr basic_slam::process_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud){
  pcl::PointCloud<pcl::PointXYZ>::Ptr sonar_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  for (const auto& point : pointcloud->points) {
    /*/
    Intensity calculation
    /*/
    sonar_cloud->push_back(point);
  }

  return sonar_cloud;
}


void basic_slam::get_odom_from_ros(){
  _sensor_position[0] = odometry->pose.pose.position.x;
  _sensor_position[1] = odometry->pose.pose.position.x;
  _sensor_position[2] = odometry->pose.pose.position.x;
  
  _sensor_rotation_quaternion[0] = odometry->pose.pose.orientation.w;
  _sensor_rotation_quaternion[1] = odometry->pose.pose.orientation.x;
  _sensor_rotation_quaternion[2] = odometry->pose.pose.orientation.y;
  _sensor_rotation_quaternion[3] = odometry->pose.pose.orientation.z;
}


void basic_slam::process_imput()
{
  bool alltopic = true;
  for(bool check : topic_check){
    if(check ==false){alltopic = false;}
  }
  
  if(alltopic){
    std::cout << "Everything received : processing !" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*depthcam_pointcloud, *temp_cloud);
    
    get_odom_from_ros();

    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = process_pointcloud(temp_cloud);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*processed_cloud, output_msg);
    output_msg.header = depthcam_pointcloud->header;

    publisher_->publish(output_msg);
  }
}
