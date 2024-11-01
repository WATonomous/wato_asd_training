#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ControlNode::timer_callback, this));
}

void ControlNode::timer_callback() {
  char sample_str[100];
  sprintf(sample_str, "Hello WATO! %lu", std::chrono::high_resolution_clock::now().time_since_epoch() / std::chrono::milliseconds(1));

  auto msg = std_msgs::msg::String();
  msg.data = sample_str;

  publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Hello WATO! %lu", std::chrono::high_resolution_clock::now().time_since_epoch() / std::chrono::milliseconds(1));
}

void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odom) {
  RCLCPP_INFO(this->get_logger(), "robot odom (x,y,z): %f,%f,%f", msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
