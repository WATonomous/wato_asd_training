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
    void subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

  private:
    robot::ControlCore control_;

    // goal point subscriber
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;

    // ROS2 publisher sending string messages to a topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Goal point
    geometry_msgs::msg::PointStamped goal_point_;
    bool goal_point_received_;

    float Kp_linear = 0.5;
    float Kp_angular = 0.5;

    // Transform buffer and listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif
