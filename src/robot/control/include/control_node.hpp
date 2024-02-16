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
#include "nav_msgs/msg/odometry.h"
#include "nav_msgs/msg/odometry.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();


  private:
    robot::ControlCore control_;

    void subscription_callback(const  nav_msgs::msg::Odometry::SharedPtr msg);
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
};

#endif
