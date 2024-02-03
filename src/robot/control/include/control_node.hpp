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

std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr goal_point_publisher_;
rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_subscriber_;
rclcpp::TimerBase::SharedPtr timer_;

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    void timer_callback();
    void goal_point_callback(geometry_msgs::msg::PointStamped::SharedPtr msg);
    
  private:
    robot::ControlCore control_;

    geometry_msgs::msg::PointStamped transformed_point;
    geometry_msgs::msg::PointStamped goal_point;
};

#endif
