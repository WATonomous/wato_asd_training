#include <memory>

#include "control_node.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/example_string", 10);

  timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ControlNode::timer_callback, this));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 10,std::bind(&ControlNode::odometry_callback, this, std::placeholders::_1));
}

void ControlNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello World Message";
  string_pub_->publish(message);

  // RCLCPP_INFO(this->get_logger(), "Hello World Test 321");  
}

void ControlNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

  auto position = msg->pose.pose.position;
  RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f, z: %.2f", position.x, position.y, position.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
