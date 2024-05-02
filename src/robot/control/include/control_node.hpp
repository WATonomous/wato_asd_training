#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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

private:
    robot::ControlCore control_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;
    void subscriber_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    // TF2 Elements
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publisher for control commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Current goal point
    geometry_msgs::msg::PointStamped current_goal_;

    const double Kp_linear = 0.5;
    const double Kp_angular = 0.5;
};

#endif  // CONTROL_NODE_HPP_
