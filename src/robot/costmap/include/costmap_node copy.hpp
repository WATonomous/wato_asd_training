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
    void publishMessage();
 
  private:
    robot::CostmapCore costmap_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    // rclcpp::TimerBase::SharedPtr timer_;
    void lidar_callback(sensor_msgs::msg::LaserScan msg);
    const float map_size_;
    const float resolution_;
    const int origin_index_;
    const int num_rows_;
    const int num_cols_;
    // const float inflation_radius_metres_;
    // const float inflation_radius_; // measured in cells
    const float max_cost = 85;
    std::vector<std::vector<int>> matrix;
    std::pair<int, int> compute_grid_indices(float range, float angle);
};
 
#endif