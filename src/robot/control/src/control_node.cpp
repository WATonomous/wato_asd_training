#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  subscriber_= this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
}

void ControlNode::subscription_callback(nav_msgs::msg::Odometry::SharedPtr msg){
  auto position_x = msg->pose.pose.position.x;
  auto position_y = msg->pose.pose.position.y;
  auto position_z = msg->pose.pose.position.z;

  // Print the pose information
  RCLCPP_INFO(this->get_logger(), "Here: Position (x, y, z): (%f, %f, %f)", position_x, position_y, position_z);

}
void ControlNode::timer_callback(){
    // RCLCPP_INFO(this->get_logger(), "Hello World!");
     // Publish a string message
  auto message = std_msgs::msg::String();
  message.data = "Publishing a string message!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}