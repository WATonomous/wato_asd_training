#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/test_topic", 20);
  publish();
}

void ControlNode::publish() {
  publisher_->publish(geometry_msgs::msg::Pose());
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
