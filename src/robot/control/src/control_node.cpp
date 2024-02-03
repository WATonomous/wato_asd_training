#include <memory>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  msg_pub_ = 
    this->create_publisher<std_msgs::msg::String>(
      "/example_string",
      20
    );
  
  odom_subscriber_ = 
    this->create_subscription<nav_msgs::msg::Odometry>(
      "/model/robot/odometry",
      20,
      std::bind(&ControlNode::odom_callback, this, std::placeholders::_1)
    );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), 
    std::bind(&ControlNode::timer_callback, this));
}

void ControlNode::timer_callback() {
  // RCLCPP_INFO(this->get_logger(), "Hello World...");

  auto msg = std_msgs::msg::String();
  msg.data = "Hello example_string topic...";
  msg_pub_->publish(msg);
}

void ControlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto xpos = msg->pose.pose.position.x;
  auto ypos = msg->pose.pose.position.y;
  auto zpos = msg->pose.pose.position.z;

  RCLCPP_INFO(this->get_logger(), "Position co-ords: x=%f, y=%f, z=%f", xpos, ypos, zpos);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
