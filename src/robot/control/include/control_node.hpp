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
#include "nav_msgs/msg/odometry.hpp"
#include "sample_msgs/msg/unfiltered.hpp"
#include "std_msgs/msg/string.hpp"
class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    

  private:
    robot::ControlCore control_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr geo_subscriber_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    geometry_msgs::msg::PointStamped::SharedPtr new_goal_point_ ;
    double Kp_linear{0.5};
    double Kp_angular{0.5};

    void timer_callback();
    void subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void geo_subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
};

#endif
