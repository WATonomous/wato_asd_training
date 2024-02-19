#include <memory>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlCore::timer_callback, this));
}

void ControlCore::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "generic message i wrote\n");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}


