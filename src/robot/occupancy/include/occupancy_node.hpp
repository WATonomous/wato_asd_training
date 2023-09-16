#ifndef OCCUPANCY_NODE_HPP_
#define OCCUPANCY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "occupancy_core.hpp"

class OccupancyNode : public rclcpp::Node {
  public:
    /**
    * Occupancy node constructor.
    */
    OccupancyNode(float map_resolution);

  private:
    // ROS2 Subscription node callback used to process laserscan data coming 
    // from the lidar msg
    void laserscan_callback_(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // ROS2 Subscription node callback used to process pose data coming 
    // from the Odometry msg
    void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Retrieves the current time
    rclcpp::Time get_time_();

    // Subscribes to the Lidar Topic which provides a laser scan
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;

    // TF readers
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishes Occupancy Grid
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;

    // Core logic for producing the occupancy grid
    robot::OccupancyCore occupancy_;
};

#endif 
