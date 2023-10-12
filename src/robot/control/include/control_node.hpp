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
#include "std_msgs/msg/string.hpp"

rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    void example_callback(std_msgs::msg::String::SharedPtr msg);
    void timer_callback();
    

  private:
    robot::ControlCore control_;
    
    
    
    

};

#endif
