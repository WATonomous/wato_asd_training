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

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();

    void subscription_callback(
      const geometry_msgs::msg::Pose::SharedPtr msg);
 
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr raw_sub_;

    // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr control_pub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_pub_;

};

#endif