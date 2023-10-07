#include <memory>

#include "control_node.hpp"

void ControlNode::timer_callback(){
  // Deliverable 5.1
  RCLCPP_INFO(get_logger(), "Timer for 5.1");

  // Deliverable 5.2
  auto msg = std_msgs::msg::String();
  msg.data = "Publish for 5.2";
  publisher_->publish(msg);
}

// Deliverable 5.3
void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto x = msg->pose.pose.position.x;
  auto y = msg->pose.pose.position.y;
  auto z = msg->pose.pose.position.z;
  //RCLCPP_INFO(get_logger(), "X: %f | Y: %f | Z: %f", x, y, z);
}

// Deliverable 6.1
void ControlNode::goalsub_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_point = *msg;
  auto x = msg->point.x;
  auto y = msg->point.y;
  auto z = msg->point.z;
  RCLCPP_INFO(get_logger(), "Goal Point: \nX: %f | Y: %f | Z: %f", x, y, z);
}

void ControlNode::movetimer_callback() {
  //RCLCPP_INFO(get_logger(), "Timer Test");

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }
  auto transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(goal_point, transformed_point, transform);

  geometry_msgs::msg::Twist msg = geometry_msgs::msg::Twist();
  msg.linear.x = transformed_point.point.x * Kp_linear;
  msg.angular.z = transformed_point.point.y * Kp_angular;
  twistpublisher_->publish(msg);
}

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  subscriber_= this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  goalsub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::goalsub_callback, this, std::placeholders::_1));
  
  movetimer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::movetimer_callback, this));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  twistpublisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
