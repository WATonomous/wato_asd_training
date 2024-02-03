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
    
  private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    robot::ControlCore control_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr goal_point_publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PointStamped transformed_point;

    void timer_callback();
    // void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void goal_point_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);
};

#endif
