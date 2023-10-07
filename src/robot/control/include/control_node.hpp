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
#include "std_msgs/msg/string.h"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

    // Deliverable 5.1
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    // Deliverable 5.2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // Deliverable 5.3
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    void subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Deliverable 6.1
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goalsub_;
    void goalsub_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    // Deliverable 6.2
    rclcpp::TimerBase::SharedPtr movetimer_;
    void movetimer_callback();
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PointStamped goal_point;

    // Deliverable 6.3
    double Kp_linear = 0.5;
    double Kp_angular = 0.5;

    // Deliverable 6.4
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twistpublisher_;


  private:
    robot::ControlCore control_;
};

#endif
