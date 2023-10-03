#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  //5.1
  timer_1 = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));

  //5.2
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);

  //5.3
  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, std::bind(&ControlNode::odometry_callback, this, std::placeholders::_1));

  //6.1
  pointstamped_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::pointstamped_callback, this, std::placeholders::_1));

  //6.2
  timer_2 = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::proportional_controller_callback, this));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

//5.1
void ControlNode::timer_callback(){
  auto message = std_msgs::msg::String();
  message.data = "Hello, world!";
  //5.2
  publisher_->publish(message);
  RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
}

//5.3
void ControlNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;

  RCLCPP_INFO(this->get_logger(), "Received odometry message. Coordinates: x=%f, y=%f, z=%f", x, y, z);
}

//6.1
void ControlNode::pointstamped_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_point_ = *msg;
  double x = msg->point.x;
  double y = msg->point.y;
  double z = msg->point.z;

  RCLCPP_INFO(this->get_logger(), "Received pointstamped message. Coordinates: x=%f, y=%f, z=%f", x, y, z);
}

//6.2
void ControlNode::proportional_controller_callback(){
  geometry_msgs::msg::TransformStamped transform;

  try {
      transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(goal_point_, transformed_point, transform);

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
