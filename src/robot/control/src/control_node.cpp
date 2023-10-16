#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  // //Deliverable 5.3
  // subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "/model/robot/odometry", 20, 
  //   std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  // //Deliverable 5.2
  // publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  // //Deliverable 5.1
  // timer_ = this->create_wall_timer(
  //   std::chrono::milliseconds(1000),
  //   std::bind(&ControlNode::timer_callback, this)); 
  
  //Deliverable 6.1
  subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 20,
    std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::timer_callback, this));
  
  
}

void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  //Deliverable 6.1
  double x = msg->point.x;
  double y = msg->point.y;
  double z = msg->point.z;
  RCLCPP_INFO(this->get_logger(), "Position: x: %f, y: %f, z: %f", x, y, z);
}

void ControlNode::timer_callback()
{
  //Deliverable 6.1
  RCLCPP_INFO(this->get_logger(), "Timer Message"); 
}

// void ControlNode::timer_callback()
// {
//   //Deliverable 5.1
//   RCLCPP_INFO(this->get_logger(), "Timer Message");
//   //Deliverable 5.2
//   auto msg = std_msgs::msg::String();
//   msg.data = "Publishing Message";
//   publisher_->publish(msg);
  
// }

// void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
//   //Deliverable 5.3
//   double x = msg->pose.pose.position.x;
//   double y = msg->pose.pose.position.y;
//   double z = msg->pose.pose.position.z;
//   RCLCPP_INFO(this->get_logger(), "Position: x: %f, y: %f, z: %f", x, y, z);
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
