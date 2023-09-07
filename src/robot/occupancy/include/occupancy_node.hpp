#ifndef OCCUPANCY_NODE_HPP_
#define OCCUPANCY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "occupancy_core.hpp"

class OccupancyNode : public rclcpp::Node {
  public:
    /**
    * Occupancy node constructor.
    */
    OccupancyNode();

  private:
    // ROS2 Subscription node callback used to process laserscan data coming 
    // from the lidar msg
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // ROS2 Subscription node callback used to process pose data coming 
    // from the robot pose msg
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Subscribes to the Lidar Topic which provides a laser scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    // Subscribes to the Pose Topic which provides a Pose of the robot
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

    // Publishes Occupancy Grid
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;

    // Core logic for producing the occupancy grid
    geometry_msgs::msg::Pose latest_pose_;

    // Core logic for producing the occupancy grid
    robot::OccupancyCore occupancy_;
};

#endif 
