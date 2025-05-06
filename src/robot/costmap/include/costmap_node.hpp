#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode(double resolution, int width, int height, int inflation_radius, int max_cost); 
 
  private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    void initCostmap();

    void convert2Grid(double range, double angle, int& x_grid, int& y_grid);

    void markObstacle(int x_grid, int y_grid);

    double dist2Point(int x1, int y1, int x2, int y2);

    double getCost(int x, int y, int new_x, int new_y);

    void inflateObstacles();

    void publishCostmap();

    // costmap variables
    robot::CostmapCore costmap_;
    std::vector<std::vector<int>> costmap_grid_;

    // parameters
    double resolution_;
    int width_;
    int height_;
    int inflation_radius_;
    int max_cost_;

    // publishers and subscribers and ros2 messages
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    nav_msgs::msg::OccupancyGrid costmap_msg_;
};
 
#endif  