#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(int delay_ms): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(delay_ms), std::bind(&ControlNode::timer_callback, this));
  
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", BUFFER_SIZE);
  
  subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", BUFFER_SIZE, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));

  goal_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", BUFFER_SIZE, std::bind(&ControlNode::goal_point_subscription_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ControlNode::timer_callback()
{
  auto transformed_point = geometry_msgs::msg::PointStamped();

  try {
    transform_ = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);

    tf2::doTransform(goal_point_, transformed_point, transform_);

    RCLCPP_INFO(this->get_logger(), "Target point transformed to robot frame -> x: %.2f, y: %.2f, z: %.2f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
    return;
  }

  auto message = geometry_msgs::msg::Twist();
  double v_linear = kp_linear_ * transformed_point.point.x;
  double v_angular = kp_angular_ * transformed_point.point.y;

  // saturation
  v_linear = std::max(-2.0, std::min(v_linear, 2.0));
  v_angular = std::max(-1.0, std::min(v_angular, 1.0));

  message.linear.x = v_linear;
  message.angular.z = v_angular;

  RCLCPP_INFO(this->get_logger(), "Adjusting velocities -> linear: %.2f, angular: %.2f", v_linear, v_angular);
  publisher_->publish(message);
}

void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double linear_x = msg->twist.twist.linear.x;
  double linear_y = msg->twist.twist.linear.y;
  double linear_z = msg->twist.twist.linear.z;
  // RCLCPP_INFO(this->get_logger(), "Odometry Linear Velocity -> x: %.2f, y: %.2f, z: %.2f", linear_x, linear_y, linear_z);
}

void ControlNode::goal_point_subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Target point updated -> x: %.2f, y: %.2f, z: %.2f", msg->point.x, msg->point.y, msg->point.z);
  goal_point_ = *msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(1000));
  rclcpp::shutdown();
  return 0;
}
