#include <chrono>
#include <memory>

#include "occupancy_node.hpp"

OccupancyNode::OccupancyNode() 
  : Node("occupancy"), occupancy_(robot::OccupancyCore()) 
{
  // Initialize ROS2 Constructs
  lidar_sub_ = 
  this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&OccupancyNode::laserscan_callback, this, std::placeholders::_1)
  );

  pose_sub_ = 
  this->create_subscription<nav_msgs::msg::Odometry>(
    "/model/robot/pose", 10, std::bind(&OccupancyNode::pose_callback, this, std::placeholders::_1)
  );

  occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/occupancy", 10
  );
}

void OccupancyNode::pose_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Odometry gives pose with covariance (->pose), we only need 
  // its pose (therefore ->pose.pose)
  latest_pose_ = msg->pose.pose;
}

void OccupancyNode::laserscan_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
  nav_msgs::msg::OccupancyGrid occupancy_msg;
  occupancy_msg.data = occupancy_.get_occupancy_data(msg);
  occupancy_msg.info = occupancy_.get_meta_map_data();
  occupancy_msg.info.origin = latest_pose_;

  occupancy_pub_->publish(occupancy_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyNode>());
  rclcpp::shutdown();
  return 0;
}
