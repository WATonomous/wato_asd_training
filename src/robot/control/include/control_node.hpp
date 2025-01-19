#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <cmath>

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void computeControlCommands();

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Pose current_pose_;
    bool path_received_;
    bool odom_received_;

    // Control parameters
    double lookahead_distance_;
    double max_linear_speed_;
    double max_angular_speed_;
};

#endif
