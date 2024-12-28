#include <memory>

#include "control_node.hpp"

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp" 

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{

  Kp_angular_ = 0.5;
  Kp_linear_ = 0.5;

  string_pub_ = this->create_publisher<std_msgs::msg::String>("/example_string", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);


  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::control_loop_callback, this));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 10, std::bind(&ControlNode::odometry_callback, this, std::placeholders::_1));

  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10, std::bind(&ControlNode::goal_point_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

void ControlNode::goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Goal Point Received -> x: %.2f, y: %.2f, z: %.2f", msg->point.x, msg->point.y, msg->point.z);
  have_goal_ = true;
  goal_point_ = *msg;
}

void ControlNode::control_loop_callback()
{
   if (!have_goal_) {
   return;
 }

 geometry_msgs::msg::TransformStamped transform;
 try {
   transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
 }
 catch (const tf2::TransformException &ex) {
   RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
   return;
 }

 geometry_msgs::msg::PointStamped transformed_point;
 try {
   tf2::doTransform(goal_point_, transformed_point, transform);

   RCLCPP_INFO(this->get_logger(), "Transformed Goal (robot frame) -> x: %.2f, y: %.2f, z: %.2f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
 }
 catch (const tf2::TransformException &ex) {
   RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
   return;
 }

  double command_linear = Kp_linear_ * transformed_point.point.x;
  double command_angular = Kp_angular_ * transformed_point.point.y;

  geometry_msgs::msg::Twist cmd_vel;
  
  cmd_vel.linear.x = command_linear;
  cmd_vel.angular.z = command_angular;

  cmd_vel_pub_->publish(cmd_vel);

  RCLCPP_INFO(this->get_logger(), "Published cmd_vel -> linear.x: %.2f, angular.z: %.2f", cmd_vel.linear.x, cmd_vel.angular.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
