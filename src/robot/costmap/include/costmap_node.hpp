#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "costmap_core.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan);
    void inflate(std::vector<int8_t>& grid, int res, double radius);
    void set(std::vector<int8_t>& grid, int res, int y, int x, int8_t val);
    int8_t get(std::vector<int8_t> grid, int res, int y, int x);
    void setPose(const nav_msgs::msg::Odometry::SharedPtr odom);
    void print(std::string s);
    
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    double robot_angle;
    double robot_angle_x;
    double robot_angle_y;
    double robot_angle_z;
    double robot_angle_w;
    double robot_x;
    double robot_y;
    double robot_z;

    rclcpp::Time past_;
};
 
#endif 