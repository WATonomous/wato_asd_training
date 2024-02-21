#include <memory>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  Kp_linear = 0.5;
  Kp_angular = 0.5;
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  subscriber_= this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 20, 
    std::bind(&ControlNode::subscription_callback, this, 
    std::placeholders::_1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ControlNode::subscription_callback(geometry_msgs::msg::PointStamped msg)
{
  auto position_x = msg.point.x;
  auto position_y = msg.point.y;
  auto position_z = msg.point.z;
  goal_point = msg.point;

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
    tf2::doTransform(goal_point, transformed_point, transform);

    auto position2_x = transformed_point.x;
    auto position2_y = transformed_point.y;
    auto position2_z = transformed_point.z;
    RCLCPP_INFO(this->get_logger(), "Original: %f, %f, %f  - Transformed: : %f, %f, %f", position_x, position_y, position_z, position2_x, position2_y, position2_z);
    Kp_angular *= position2_y;
    Kp_linear *= position2_x;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

}

void ControlNode::timer_callback()
{
  geometry_msgs::msg::Twist messageToSend;
  messageToSend.linear.x = Kp_linear;
  messageToSend.angular.z = Kp_angular;
  publisher_->publish(messageToSend);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}