#include <memory>

#include "control_node.hpp"


ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  subscriber_= this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::control_callback, this));
}

void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_point = *msg;
  RCLCPP_INFO(this->get_logger(), "Position -> x:%f, y%f, z%f", msg->point.x, msg->point.y, msg->point.z);
}

void ControlNode::control_callback(){

  geometry_msgs::msg::TransformStamped transform;
 
  try {
      transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(goal_point, transformed_point, transform);

  RCLCPP_INFO(this->get_logger(), "Goal Point -> x:%f, y%f, z%f", goal_point.point.x, goal_point.point.y, goal_point.point.z);

  float c_linear = Kp_linear * transformed_point.point.x;
  float c_angular = Kp_angular * transformed_point.point.y;

  geometry_msgs::msg::Twist controls;
  controls.linear.x = c_linear;
  controls.angular.z = c_angular;

  publisher_->publish(controls);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
