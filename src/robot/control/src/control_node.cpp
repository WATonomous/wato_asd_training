#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore()){

  subscriber_ = this->create_subscription<nav_msgs::msg::Odometry >(
    "/model/robot/odometry", 20, 
    std::bind(&ControlNode::subscription_callback, this, 
    std::placeholders::_1));
  //setup timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), std::bind(&ControlNode::timer_callback, this));

  publisher_ = this->create_publisher<std_msgs::msg::String>("/walkout", 20);
  auto msg = std_msgs::msg::String();
  msg.data = "setup test ";
  publisher_->publish(msg);
}

void ControlNode::timer_callback(){
  
    auto message = std_msgs::msg::String();
    message.data = "timer message";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    
    //rclcpp::TimerBase::SharedPtr timer_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    //size_t count_;
}



void ControlNode::subscription_callback(const  nav_msgs::msg::Odometry::SharedPtr msg)
{
	// Print to the console
    auto position = msg->pose.pose.position;
    
    RCLCPP_INFO(this->get_logger(), "Position: [x: %.3f, y: %.3f, z: %.3f]",
                position.x, position.y, position.z);
}




int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

/*
ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}*/
