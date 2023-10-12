#include <memory>
#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore())
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 160);
  subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "/example_string", 160,
    std::bind(&ControlNode::example_callback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
}

void ControlNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "This is a timer callback");
  auto msg = std_msgs::msg::String();
  msg.data = "This is a published message";
  publisher_->publish(msg);
}

void ControlNode::example_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "This is a subscription callback: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}