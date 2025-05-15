#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
 
// Constructor for the CostmapNode class
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  resolution_ = 0.1;
  width_ = 100;
  height_ = 100;
  origin_.position.x = 5.0;
  origin_.position.y = 5.0;
  origin_.orientation.w = 1.0;
  inflation_radius_ = 1.0;
  max_cost_ = 100;
  
  // Initialize the constructs and their parameters
  sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));
  pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  costmap_.initializeCostmap(resolution_, width_, height_, origin_, inflation_radius_, max_cost_);

  RCLCPP_INFO(this->get_logger(), "Costmap Node initialized");
}
 
// Callback function to process the laser scan data
void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.processLaserScan(msg);
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_msg = costmap_.publishCostmap();
  costmap_msg->header = msg->header;
  pub_->publish(*costmap_msg);
  RCLCPP_INFO(this->get_logger(), "Costmap published");
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}