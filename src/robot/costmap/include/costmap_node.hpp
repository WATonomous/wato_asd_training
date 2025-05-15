#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
 
  private:
    robot::CostmapCore costmap_;

    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;

    // Grid parameters
    double resolution_; // meters per cell
    int width_;
    int height_;
    geometry_msgs::msg::Pose origin_;
    double inflation_radius_;
    int max_cost_;
};
 
#endif 