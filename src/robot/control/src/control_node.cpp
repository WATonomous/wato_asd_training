#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/control_test", 20);
  
}

void ControlNode::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Publishing to Topic");
  auto message = std_msgs::msg::String();
  message.data = "This is a Test";
  publisher_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
