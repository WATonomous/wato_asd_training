#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    void timer_callback();
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  private:
    robot::ControlCore control_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_subscriber_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double Kp_linear;
    double Kp_angular;
    bool goal_point_received;

    geometry_msgs::msg::PointStamped transformed_goal_point_;
    geometry_msgs::msg::PointStamped original_goal_point_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;
};

#endif
