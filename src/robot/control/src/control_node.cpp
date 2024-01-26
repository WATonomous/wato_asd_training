#include <memory>

#include "control_node.hpp"
#include <functional>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  // deliverable 5
  string_publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  odom_subscriber_= this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, 
      std::bind(&ControlNode::printOdomInfo, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ControlNode::timer_callback, this));

  // deliverable 6
  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20,
    std::bind(&ControlNode::goalCallback, this, std::placeholders::_1));
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlTimerCallback, this));

  // deliverable 6.2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ControlNode::timer_callback()
{
  // deliverable 5.1
  RCLCPP_INFO(this->get_logger(), "Hello world");

  // deliverable 5.2
  std_msgs::msg::String msg; 
  msg.data = "Hello world 2";
  string_publisher_->publish(msg);
}

// deliverable 5.3
void ControlNode::printOdomInfo(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", 
    msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

// deliverable 6.1
void ControlNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", 
    msg->point.x, msg->point.y, msg->point.z);
  goal_point_ = *msg;
}

// deliverable 6.3 and 6.4
void ControlNode::controlTimerCallback()
{
  RCLCPP_INFO(this->get_logger(), "hello");

  //transform from world to robot
  geometry_msgs::msg::TransformStamped transform;

  try {
      transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_goal_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(goal_point_, transformed_goal_point, transform);

  // publish twist
  geometry_msgs::msg::Twist msg;
  msg.linear.x = kp_linear_ * transformed_goal_point.point.x;
  msg.angular.z = kp_angular_ * transformed_goal_point.point.y;

  twist_publisher_->publish(msg);
}
