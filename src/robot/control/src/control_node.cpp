#include <memory>

#include "control_node.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/time.h>
#include "tf2/exceptions.h"

#include "std_msgs/msg/string.hpp"

#include <cmath>

// #include "nav_msgs/msg/odometry.hpp"


ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore()), count_(0)
{ 
  
  Kp_linear_ = 0.5;
  Kp_angular_ = 0.5;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  raw_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 20,
    std::bind(
      &ControlNode::point_callback, this, 
      std::placeholders::_1));

  control_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));

}

void ControlNode::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{

  auto original_point = geometry_msgs::msg::PointStamped();
  // auto transformed_point = geometry_msgs::msg::PointStamped();
  // geometry_msgs::msg::TransformStamped transform;

  original_point.point.x = msg->point.x;
  original_point.point.y = msg->point.y;
  original_point.point.z = msg->point.z;

  RCLCPP_INFO(this->get_logger(), "Received point: (x: %.2f, y: %.2f, z: %.2f)", 
  original_point.point.x, original_point.point.y, original_point.point.z);

  goal_point_ = original_point;

  // RCLCPP_INFO(this->get_logger(), "Defining Transform from sim_world to robot");

  // try {
  //   transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  // } catch (const tf2::TransformException & ex) {
  //   RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  // }

  // tf2::doTransform(original_point, transformed_point, transform);

  // RCLCPP_INFO(this->get_logger(), "Transformed point: (x: %.2f, y: %.2f, z: %.2f)", 
  // transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

  // goal_point_ = transformed_point;
}

void ControlNode::timer_callback()
{

  if (goal_point_ == NULL) return;

  geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::Twist velocity_command;
  auto transformed_point = geometry_msgs::msg::PointStamped();
  double linear_v;
  double angular_v;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  tf2::doTransform(goal_point_, transformed_point, transform);

  RCLCPP_INFO(this->get_logger(), "Transformed point: (x: %.2f, y: %.2f, z: %.2f)", 
  transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

  if (abs(transformed_point.point.x) < 0.05) {
    linear_v = 0;
  } else {
    
    linear_v = Kp_linear_ * transformed_point.point.x;
  }

   if (abs(transformed_point.point.y) < 0.05) {
    angular_v = 0;
  } else {
    angular_v = Kp_angular_ * transformed_point.point.y;
  }

  RCLCPP_INFO(this->get_logger(), "Linear velocity: %.2f Angular velocity: %.2f", linear_v, angular_v);

  velocity_command.linear.x = linear_v;
  velocity_command.angular.z = angular_v;

  RCLCPP_INFO(this->get_logger(), "Publishing velocity command");

  control_pub_->publish(velocity_command);

  // count_ ++;
  // RCLCPP_INFO(this->get_logger(), "Timer called %d times.", count_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}