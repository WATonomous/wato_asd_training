#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/msg/string.h"
#include "sample_msgs/msg/unfiltered.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void timer_callback();

  private:
    // PID Controller
    double Kp_linear = 0.5;
    double Kp_angular = 0.5;

    // Transforms
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PointStamped::SharedPtr new_goal_point;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscriber_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Subscription callback
    void subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    // Core
    robot::ControlCore control_;
};

#endif
