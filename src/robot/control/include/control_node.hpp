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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"


class ControlNode : public rclcpp::Node
{
public:
  ControlNode();

private:
/*
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
void subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_callback();
  int count_;
  */
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
  void subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;

  geometry_msgs::msg::PointStamped::SharedPtr goal_point_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  robot::ControlCore control_;
};

#endif
