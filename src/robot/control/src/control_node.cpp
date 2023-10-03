#include <memory>
#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
      std::bind(&ControlNode::timer_callback, this));

    publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
    
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/model/robot/odometry", 20, std::bind(&ControlNode::sub_callback, this, std::placeholders::_1));

}

void ControlNode::sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "x = %f , y = %f , z = %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void ControlNode::timer_callback(){
  //Deliverable 5.1 code RCLCPP_INFO(this->get_logger(), "Tron on top");
  auto msg = std_msgs::msg::String();
  msg.data = "Tron on top again";
  publisher_->publish(msg);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
