#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, 
  std::bind(
    &CostmapNode::lidar_callback, this,
    std::placeholders::_1));
}


void CostmapNode::lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr scan) 
{
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg = costmap_.computeCostMap(scan);
  costmap_msg->header = scan->header;
  costmap_pub_->publish(*costmap_msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}