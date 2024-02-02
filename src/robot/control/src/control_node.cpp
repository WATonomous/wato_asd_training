#include <memory>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  msg_pub_ = 
    this->create_publisher<std_msgs::msg::String>("/example_string", 20);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), 
    std::bind(&ControlNode::timer_callback, this));
}

void ControlNode::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Hello World...");

  auto msg = std_msgs::msg::String();
  msg.data = "Hello example_string topic...";
  msg_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
