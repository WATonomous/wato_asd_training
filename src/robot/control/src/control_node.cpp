#include <memory>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode(int delay_ms): Node("control"), control_(robot::ControlCore()) {
  timer_ = this->create_wall_timer(std::chrono::milliseconds(delay_ms), std::bind(&ControlNode::timer_callback, this));

  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);

  subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
}

void ControlNode::timer_callback() {
  // RCLCPP_INFO(this->get_logger(), "log time");

  auto msg = std_msgs::msg::String();
  msg.data = "this works!!!";
  publisher_->publish(msg);
}

void ControlNode::subscription_callback(nav_msgs::msg::Odometry::SharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "position: (%f %f %f)", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(2000));
  rclcpp::shutdown();
  return 0;
}
