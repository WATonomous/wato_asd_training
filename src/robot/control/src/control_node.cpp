#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_1000_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&ControlNode::timer_1000_callback, this));

  timer_100_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::timer_100_callback, this));

  publisher_es_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);

  publisher_control_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  subscriber_cl_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/model/robot/odometry", 20,
      std::bind(&ControlNode::subscription_callback_current_location, this,
      std::placeholders::_1));

  subscriber_sl_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 20,
      std::bind(&ControlNode::subscription_callback_selected_location, this,
      std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ControlNode::timer_1000_callback() 
{
  auto msg = std_msgs::msg::String();
  msg.data = "Publishing message";
  publisher_es_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "1s message");
}

void ControlNode::timer_100_callback() 
{
  if (!this->goal_point) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();

  tf2::doTransform(*this->goal_point, transformed_point, transform);
  auto msg = geometry_msgs::msg::Twist();

  float kp_linear = 0.5;
  float kp_angular = 0.5;

  msg.linear.x = kp_linear*transformed_point.point.x;
  msg.angular.z = kp_angular*transformed_point.point.y;
  publisher_control_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Linear: %f, Angular: %f", msg.linear.x, msg.angular.z);
}

void ControlNode::subscription_callback_current_location(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  float z = msg->pose.pose.position.z;

  RCLCPP_INFO(this->get_logger(), "Current coordinates: (%f, %f, %f)", x, y, z);
}

void ControlNode::subscription_callback_selected_location(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  this->goal_point = msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
