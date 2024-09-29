#include <memory>

#include "control_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/string.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  raw_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "/pose_topic", 20,
    std::bind(
      &ControlNode::subscription_callback, this, 
      std::placeholders::_1));

  control_pub_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&ControlNode::timer_callback, this));

}

void ControlNode::subscription_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received x_position=%f", msg->position.x);
  geometry_msgs::msg::Pose modified_pose;
  // control_pub_->publish(modified_pose);
}

void ControlNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Timer callback called");
  auto message = std_msgs::msg::String();
  message.data = "HOLY SHIT 2";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  control_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}