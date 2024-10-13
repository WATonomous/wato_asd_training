#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(int delay_ms): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(delay_ms), std::bind(&ControlNode::timer_callback, this));
  
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", BUFFER_SIZE);
  
  subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", BUFFER_SIZE, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));

  goal_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", BUFFER_SIZE, std::bind(&ControlNode::goal_point_subscription_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ControlNode::timer_callback()
{
  // RCLCPP_INFO(this->get_logger(), "One timestep has passed");
  auto message = std_msgs::msg::String();
  message.data = "Hi from publisher";
  publisher_->publish(message);
}

void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double linear_x = msg->twist.twist.linear.x;
  double linear_y = msg->twist.twist.linear.y;
  double linear_z = msg->twist.twist.linear.z;
  RCLCPP_INFO(this->get_logger(), "Odometry Linear Velocity -> x: %.2f, y: %.2f, z: %.2f", linear_x, linear_y, linear_z);
}

void ControlNode::goal_point_subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Target point updated -> x: %.2f, y: %.2f, z: %.2f", msg->point.x, msg->point.y, msg->point.z);

  try {
    transform_ = tf_buffer_->lookupTransform("sim_world", "robot", tf2::TimePointZero);

    auto transformed_point = geometry_msgs::msg::PointStamped();
    tf2::doTransform(*msg, transformed_point, transform_);

    transformed_goal_point_ = transformed_point;
    
    RCLCPP_INFO(this->get_logger(), "Target point transformed to robot frame -> x: %.2f, y: %.2f, z: %.2f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(100));
  rclcpp::shutdown();
  return 0;
}
