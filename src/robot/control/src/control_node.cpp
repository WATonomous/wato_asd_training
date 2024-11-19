#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  //timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::control_callback, this));

  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);

  subscriber_= this->create_subscription<nav_msgs::msg::Odometry>(
    "/model/robot/odometry", 20,
    std::bind(&ControlNode::subscription_callback, this,
    std::placeholders::_1));
  
  gp_subscriber_= this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 20,
    std::bind(&ControlNode::get_goal_point, this,
    std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

// 5.1, 5.2
void ControlNode::timer_callback()
{
  //RCLCPP_INFO(this->get_logger(), "Hello World!");
  auto message = std_msgs::msg::String();
  message.data = "Published";
  publisher_->publish(message);
}

// 5.3
void ControlNode::subscription_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
  auto x = msg->pose.pose.position.x;
  auto y = msg->pose.pose.position.y;
  auto z = msg->pose.pose.position.z;
  //RCLCPP_INFO(this->get_logger(), "Printing x,y,z coords: (%f, %f, %f)", x, y, z);
}

// 6.1
void ControlNode::get_goal_point(geometry_msgs::msg::PointStamped::SharedPtr msg) {
  auto x = msg->point.x;
  auto y = msg->point.y;
  auto z = msg->point.z;
  //RCLCPP_INFO(this->get_logger(), "Goal Point coords: (%f, %f, %f)", x, y, z);

  goal_point_ = msg;
}

// 6.2, 6.3, 6.4, 6.5
void ControlNode::control_callback() {
  if (!goal_point_) return;
  double Kp_linear = 0.5;
  double Kp_angular = 0.5;

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(*goal_point_, transformed_point, transform);

  Kp_linear *= transformed_point.point.x;
  Kp_angular *= transformed_point.point.y;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = Kp_linear;
  cmd.angular.z = Kp_angular;

  cmd_publisher_->publish(cmd);
}