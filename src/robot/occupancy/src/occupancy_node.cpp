#include <chrono>
#include <memory>

#include "occupancy_node.hpp"

OccupancyNode::OccupancyNode(float map_resolution) 
  : Node("occupancy"), occupancy_(robot::OccupancyCore(map_resolution))
{
  // Initialize ROS2 Constructs
  lidar_sub_ = 
  this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&OccupancyNode::laserscan_callback_, this, std::placeholders::_1)
  );

  occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/occupancy", 10
  );
}

void OccupancyNode::laserscan_callback_(
  const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
  nav_msgs::msg::OccupancyGrid occupancy_msg;

  // Populate Header
  occupancy_msg.header.stamp = get_time_();
  occupancy_msg.header.frame_id = msg->header.frame_id;

  // Populate Data
  occupancy_msg.info = occupancy_.get_map_meta_data();
  occupancy_msg.data = occupancy_.get_occupancy_data(msg);
  occupancy_msg.info.map_load_time = get_time_();

  // Publish
  occupancy_pub_->publish(occupancy_msg);
}

rclcpp::Time OccupancyNode::get_time_()
{
  return this->get_clock()->now();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // We set a map resolution of 0.5 here
  rclcpp::spin(std::make_shared<OccupancyNode>(1));
  rclcpp::shutdown();
  return 0;
}
