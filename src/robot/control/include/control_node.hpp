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
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
   

  private:
    robot::ControlCore control_;
    // 5.1
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // 5.2
    rclcpp::TimerBase::SharedPtr timer_;
    // 5.3
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    // 6.1
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;
    // 6.2
    rclcpp::TimerBase::SharedPtr goal_timer; 
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    // 6.3 & 6.4
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goalSubscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_publisher_;

    double Kp_linear = 0.5;
    double Kp_angular = 0.5;

    geometry_msgs::msg::PointStamped goal_point;
    
    double goal_point_transformed_x;
    double goal_point_transformed_y;
    

    // 5.1
    void timer_callback();
    // 5.3
    void subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // 6.1
    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    // 6.2
    void goal_timer_callback(); 
  
};

#endif
