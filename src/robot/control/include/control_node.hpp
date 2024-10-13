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
    ControlNode(int delay_ms);

  private:
    static constexpr int BUFFER_SIZE = 20;

    robot::ControlCore control_;

    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    void subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_subscriber_;
    void goal_point_subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    geometry_msgs::msg::PointStamped transformed_goal_point_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::TransformStamped transform_;
};


#endif
