#include <memory>

#include "control_node.hpp"

#define timer_delay 1500
ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_delay), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);

  subscriber_ = this -> create_subscription<nav_msgs::msg::Odometry>(
                        "/model/robot/odometry", 20, 
                        std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
}

void ControlNode::timer_callback(){
  auto message = std_msgs::msg::String();
  message.data = "Published message";
  publisher_ -> publish(message);
  RCLCPP_INFO(this->get_logger(), "Deliverables");
}

void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  float x = msg -> pose.pose.position.x;
  float y = msg -> pose.pose.position.y;
  float z = msg -> pose.pose.position.z;
  RCLCPP_INFO(this->get_logger(), "Coordinates (x,y,z): (%f, %f, %f)", x,y,z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
