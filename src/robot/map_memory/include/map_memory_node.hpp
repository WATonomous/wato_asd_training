#ifndef MAP_MEMORY_NODE_HPP
#define MAP_MEMORY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode();

private:
    robot::MapMemoryCore map_memory_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Global map and costmap
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;

    // Flags
    bool costmap_updated_ = false;
    bool should_update_map_ = false;
    bool map_initialized_ = false;

    int costmap_update_counter_ = 0;

    // Robot state
    double last_x_;
    double last_y_;
    double current_x_;
    double current_y_;
    double current_theta_;

    // Parameters
    const double distance_threshold_;

    // Callback functions
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

    // Map update functions
    void updateMap();
    void integrateCostmap();
};

#endif // MAP_MEMORY_NODE_HPP
