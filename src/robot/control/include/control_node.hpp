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


class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PointStamped goal_point_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    const double kp_linear_ = 0.5;
    const double kp_angular_ = 0.5;

    void timer_callback();
    void printOdomInfo(const nav_msgs::msg::Odometry::SharedPtr msg);

    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void controlTimerCallback();
};

#endif
