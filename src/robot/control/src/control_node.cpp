#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  subscriber_= this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  goalpose = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::goalpose_callback, this, std::placeholders::_1));
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::control_timer_callback, this));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void ControlNode::subscription_callback(nav_msgs::msg::Odometry::SharedPtr msg){
  auto position_x = msg->pose.pose.position.x;
  auto position_y = msg->pose.pose.position.y;
  auto position_z = msg->pose.pose.position.z;

RCLCPP_INFO(this->get_logger(), "Here: Position (x, y, z): (%f, %f, %f)", position_x, position_y, position_z);

}

void ControlNode::goalpose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_point = *msg;
  RCLCPP_INFO(this->get_logger(), "Goal Pose x=%f, y=%f, z=%f", msg->point.x, msg->point.y, msg->point.z);
  RCLCPP_INFO(this->get_logger(), "Blank timer callback");
}

void ControlNode::timer_callback(){

  RCLCPP_INFO(this->get_logger(), "This is the message!");
  auto message = std_msgs::msg::String();
  message.data = "This is a published string message!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);

}

void ControlNode::control_timer_callback(){
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();

  tf2::doTransform(goal_point, transformed_point , transform);

  RCLCPP_INFO(this->get_logger(), "Point: x=%f y=%f z=%f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = Kp_linear * transformed_point.point.x;
  twist_msg.angular.z = Kp_angular * transformed_point.point.y;
  twist_publisher_->publish(twist_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
