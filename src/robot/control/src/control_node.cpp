#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  subscriber_= this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, std::bind(&ControlNode::sub_callback, this, std::placeholders::_1));
  
}

void ControlNode::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Publishing to Topic");
  auto message = std_msgs::msg::String();
  message.data = "This is a Test";
  publisher_->publish(message);
}

void ControlNode::sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto x = msg->pose.pose.position.x;
  auto y = msg->pose.pose.position.y;
  auto z = msg->pose.pose.position.z;
  RCLCPP_INFO(this->get_logger(), "Received Odometry: x=%.2f, y=%.2f, z=%.2f", x, y, z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
