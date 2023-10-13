#include <memory>

#include "nav_node.hpp"

NavNode::NavNode(): Node("transformer"), nav_(robot::NavCore())
{
  // subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "/model/robot/odometry", 20,
  //   std::bind(&NavNode::nav_callback, this,
  //   std::placeholders::_1)
  // );
}

// void NavNode::nav_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

//   RCLCPP_INFO(this->get_logger(), "\n\nReceived X =%f \nReceived Y =%f \nReceived Z =%f",
//   msg->pose.pose.position.x, msg->pose.pose.position.y, msgs->pose.pose.position.z);
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavNode>());
  rclcpp::shutdown();
  return 0;
}
