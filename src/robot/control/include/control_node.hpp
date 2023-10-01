#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    void timer_callback();
    void control_timer_callback();
    void subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

    geometry_msgs::msg::PointStamped messages_object;



    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    double Kp_linear = 0.5;
    double Kp_angular = 0.5;
    double goal_point_transformed_x; 
    double goal_point_transformed_y;
};

#endif
