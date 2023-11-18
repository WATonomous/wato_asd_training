#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  // publisher_ = this->create_publisher<Unfiltered>("/TOPIC, 20");
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
  std::bind(&ControlNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Node Constructor");
}

ControlNode::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Timer callbacked!");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
