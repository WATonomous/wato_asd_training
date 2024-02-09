#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  int delay_ms = 1000;
  int control_loop_delay_ms = 100;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(delay_ms), std::bind(&ControlNode::timer_callback, this));
  control_timer = this->create_wall_timer(std::chrono::milliseconds(control_loop_delay_ms), std::bind(&ControlNode::control_loop_callback, this));

  mypublisher = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  subscriber_= this->create_subscription<nav_msgs::msg::Odometry>(
                                                      "/model/robot/odometry", 20, 
                                                  std::bind(&ControlNode::subscription_callback, this, 
                                                  std::placeholders::_1));
  goalpose = this->create_subscription<geometry_msgs::msg::PointStamped>(
                                                      "/goal_point", 20, 
                                                  std::bind(&ControlNode::goalpose_callback, this, 
                                                  std::placeholders::_1));                
}

void ControlNode::timer_callback(){
  auto message = std_msgs::msg::String();
  message.data = "Hello World!";
  //RCLCPP_INFO(this->get_logger(), "Hello World");
  //mypublisher->publish(message);
}

void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  //RCLCPP_INFO(this->get_logger(), "Odometry x=%f, y=%f, z=%f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void ControlNode::goalpose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Goal Pose x=%f, y=%f, z=%f", msg->point.x, msg->point.y, msg->point.z);
  goal_point = *msg;
}

void ControlNode::control_loop_callback() {
  float kp_linear = 0.5f;
  float kp_angular = 0.5f;
  geometry_msgs::msg::TransformStamped transform;

  // convert transform from sim_world frame to robot frame
  try {
      transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }
  auto transformed_point = geometry_msgs::msg::PointStamped();

  tf2::doTransform(goal_point, transformed_point, transform);

  // transform pointStamped to twistStamped
  geometry_msgs::msg::Twist twist;
  twist.linear.x = transformed_point.point.x * kp_linear;
  twist.angular.z = transformed_point.point.y * kp_angular;

  twist_publisher->publish(twist);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
