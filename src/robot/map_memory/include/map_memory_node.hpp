#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback functions
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void updateMap();
    void integrateCostmap();

    // Map memory attributes
    int map_width_;  // Number of cells in x direction
    int map_height_; // Number of cells in y direction
    double map_resolution_; // Resolution of the map memory (meters per cell)

    // Variables
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_updated_ = false;
    bool map_initialized_ = false;
    bool should_update_map_ = false;
    int costmap_update_counter_ = 0;

    // Robot state
    double current_x_;
    double current_y_;
    double current_theta_;
    double last_x_;
    double last_y_;
    double distance_threshold_;
};

#endif
