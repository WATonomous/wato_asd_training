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
#include "tf2/transform_datatypes.h" // Include if you need to use tf2::Transform


class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    void goal_point_subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped transform;
    geometry_msgs::msg::PointStamped::SharedPtr goal_point_;

    float Kp_linear = -0.1, Kp_angular = -0.05;
    float x,y;

    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;  // Correct message type
    // rclcpp::TimerBase::SharedPtr publisher_timer_;
    // void subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg);  // Correct message type
    // void publisher_timer_callback();
};

#endif
