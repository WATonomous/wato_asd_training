#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  subscriber_= this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 20, 
  std::bind(&ControlNode::subscription_callback, this,
  std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
}

void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  double x = msg->point.x;
  double y = msg->point.y;
  double z = msg->point.z;
  RCLCPP_INFO(this->get_logger(), "6.1 Goal Point: x=%.2f, y=%.2f, z=%.2f", x, y, z);

  goal_point_ = *msg;
}

void ControlNode::timer_callback(){
  // RCLCPP_INFO(this->get_logger(), "Timer callback");

  if (goal_point_.header.frame_id.empty()) {
    RCLCPP_INFO(this->get_logger(), "No goal point!");
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
      transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
      return;
  }
  auto transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(goal_point_, transformed_point, transform);

  RCLCPP_INFO(this->get_logger(), "6.2 Transform Point: x=%.2f, y=%.2f, z=%.2f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

  double control_linear = transformed_point.point.x * Kp_linear;
  double control_angular = transformed_point.point.y * Kp_angular;

  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = control_linear;
  msg.angular.z = control_angular;

  publisher_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
