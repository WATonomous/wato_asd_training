#include <memory>
#include <chrono>

#include "control_node.hpp"

ControlNode::ControlNode(int delay_ms): Node("control"), control_(robot::ControlCore()) {
  timer_ = this->create_wall_timer(std::chrono::milliseconds(delay_ms), std::bind(&ControlNode::timer_callback, this));

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ControlNode::timer_callback() {
  // RCLCPP_INFO(this->get_logger(), "log time");

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto robot_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(goal_point_, robot_point, transform);

  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = KP_LINEAR*robot_point.point.x;         // linear control
  msg.angular.z = KP_ANGULAR*robot_point.point.y;         // angular control
  publisher_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "transformed: (%f %f %f)", robot_point.point.x, robot_point.point.y, robot_point.point.z);
}

void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_point_.point = msg->point;
  RCLCPP_INFO(this->get_logger(), "goal: (%f %f %f)", msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(100));
  rclcpp::shutdown();
  return 0;
}
