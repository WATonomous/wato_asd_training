#include <memory>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode(int delay_ms): Node("control"), control_(robot::ControlCore()) {
  timer_ = this->create_wall_timer(std::chrono::milliseconds(delay_ms), std::bind(&ControlNode::timer_callback, this));

  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);

  subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
}

void ControlNode::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "log time");

  auto msg = std_msgs::msg::String();
  msg.data = "this works!!!";
  publisher_->publish(msg);
}

void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "goal: (%f %f %f)", msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(100));
  rclcpp::shutdown();
  return 0;
}
