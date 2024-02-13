#include <memory>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore()) // count_(0)
{
  /*timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 10);
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/model/robot/odometry", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
    */
    subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped> (
      "/goal_point", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
      timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));
      tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

      cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    

void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) 
{
  goal_point_ = msg;
  RCLCPP_INFO(this->get_logger(), "Received goal point message: x=%f, y=%f, z=%f", 
  msg->point.x, 
  msg->point.y,
  msg->point.z);
}



void ControlNode::timer_callback()
{
double Kp_linear = 0.5;
double Kp_angular = 0.5;
  if (!goal_point_) {
    RCLCPP_INFO(this->get_logger(), "No goal point received yet");
    return;
  }

  geometry_msgs::msg::TransformStamped transform;

    try {
      transform = tf_buffer->lookupTransform("robot", "sim_world", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
    }

    auto transform_point = geometry_msgs::msg::PointStamped();

    tf2::doTransform(*goal_point_, transform_point, transform);
  
    double linear_command = Kp_linear * transform_point.point.x;
    double angular_command = Kp_angular * transform_point.point.y;

    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_command;
    cmd_vel_msg.angular.z = angular_command;

    cmd_vel_publisher_->publish(cmd_vel_msg);
}
/*
void ControlNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "This is a message from the timer callback at: %d seconds", count_++);
  std_msgs::msg::String message;
  message.data = "Hello, this is my first ros2 message!";
  publisher_->publish(message);
}

void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received odometry message: x=%f, y=%f, z=%f", 
  msg->pose.pose.position.x, 
  msg->pose.pose.position.y,
  msg->pose.pose.position.z);
}
*/

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

