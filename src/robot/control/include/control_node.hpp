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
  //Timers
    
    
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    //Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    
    //Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;
    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    void sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    //Deliverables 6.x
    rclcpp::TimerBase::SharedPtr control_timer_;
    void control_callback();
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PointStamped goal_point;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;
    double Kp_linear = 0.5;
    double Kp_angular = 0.5;

    robot::ControlCore control_;
};

#endif
