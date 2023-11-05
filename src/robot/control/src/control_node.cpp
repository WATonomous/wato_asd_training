#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  
  /*timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&ControlNode::timer_callback, this));

  publisher_ =
    this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  
  subscriber_ =
    this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20,
    std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  */

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  goalSubscriber_ = 
    this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20,
    std::bind(&ControlNode::goalSubscription_callback, this, std::placeholders::_1));

  goalTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::goalTimer_callback, this));

  control_publisher_ = 
    this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
}

void ControlNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Hello World");
  auto msg = std_msgs::msg::String();
  msg.data = "First topic!";
  publisher_->publish(msg);
}

void ControlNode::subscription_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg) 
{
  auto x = msg->pose.pose.position.x;
  auto y = msg->pose.pose.position.y;
  auto z = msg->pose.pose.position.z;

  RCLCPP_INFO(this->get_logger(), "Received position: x=%f, y=%f, z=%f", x, y, z);
}

void ControlNode::goalSubscription_callback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_point = *msg;
  RCLCPP_INFO(this->get_logger(), "Goal point: x=%f, y=%f, z=%f", goal_point.point.x, goal_point.point.y, goal_point.point.z);

}

void ControlNode::goalTimer_callback()
{
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();

  tf2::doTransform(goal_point, transformed_point, transform);

  RCLCPP_INFO(this->get_logger(), "Goal point: x=%f, y=%f, z=%f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

  auto control_point = geometry_msgs::msg::Twist();

  control_point.linear.x = Kp_linear * transformed_point.point.x;
  control_point.angular.z = Kp_angular * transformed_point.point.y;

  control_publisher_->publish(control_point);

  RCLCPP_INFO(this->get_logger(), "Successfully published control point");

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
