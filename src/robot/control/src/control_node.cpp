#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  //  5.1
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));

  // 5.2
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);

  // 5.3
  subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/model/robot/odometry", 20, 
    std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));

  // 6.1
  point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::point_callback, this, std::placeholders::_1));

  // 6.2
  goal_timer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::goal_timer_callback, this));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // 6.3
  control_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

}

// 5.1
void ControlNode::timer_callback() {
  // 5.1
  // RCLCPP_INFO(this->get_logger(), "Hello World!");

  // 5.2
  auto msg = std_msgs::msg::String();
  msg.data = "Publishing!";
  publisher_->publish(msg);
  // RCLCPP_INFO(this->get_logger(), "PUBLISHING!!!");

}

// 5.3
void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;

  RCLCPP_INFO(this->get_logger(), "Odometry position: x: %f, y: %f, z: %f", x, y, z);
}

// 6.1
void ControlNode::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_point = *msg;
  double x = msg->point.x;
  double y = msg->point.y;
  double z = msg->point.z;

  RCLCPP_INFO(this->get_logger(), "Point position: x=%f, y=%f, z=%f", x, y, z);
}

// 6.2
void ControlNode::goal_timer_callback(){
  geometry_msgs::msg::TransformStamped transform;

  try {
      transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(goal_point, transformed_point, transform);
  RCLCPP_INFO(this->get_logger(), "Transformed Point: x: %f, y: %f, z: %f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z );
  // 6.3 & 6.4
  auto control_point = geometry_msgs::msg::Twist();
  control_point.linear.x = Kp_linear * transformed_point.point.x;
  control_point.angular.z = Kp_angular * transformed_point.point.y;
  control_publisher_->publish(control_point);
  RCLCPP_INFO(this->get_logger(), "PUBLISHED!!!");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
