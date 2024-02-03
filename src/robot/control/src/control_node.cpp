#include <memory>
#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  // Deliverable 5.x
  // msg_pub_ = 
  //   this->create_publisher<std_msgs::msg::String>(
  //     "/example_string",
  //     20
  //   );
  
  // odom_subscriber_ = 
  //   this->create_subscription<nav_msgs::msg::Odometry>(
  //     "/model/robot/odometry",
  //     20,
  //     std::bind(&ControlNode::odom_callback, this, std::placeholders::_1)
  //   );
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&ControlNode::timer_callback, this)
    );
  
  goal_point_publisher_ = 
    this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      20
    );

  goal_point_subscriber_ = 
    this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point",
      20,
      std::bind(&ControlNode::goal_point_callback, this, std::placeholders::_1)
    );
}

void ControlNode::timer_callback() {
  const double Kp_linear = 0.5;
  const double Kp_angular = 0.1;

  geometry_msgs::msg::Twist control_signal;

  control_signal.linear.x = Kp_linear * transformed_point.point.x;
  control_signal.angular.z = Kp_angular * transformed_point.point.y;

  goal_point_publisher_->publish(control_signal);

  RCLCPP_INFO(this->get_logger(), "Linear Vel: %f, Angular Vel: %f", control_signal.linear.x, control_signal.angular.z);
}

void ControlNode::goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  auto xpos = msg->point.x;
  auto ypos = msg->point.y;
  auto zpos = msg->point.z;

  RCLCPP_INFO(this->get_logger(), "Goal co-ords: [x,y,z] = [%f, %f, %f]", xpos, ypos, zpos);

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  tf2::doTransform(*msg, transformed_point, transform);
}

// Deliverable 5.x
// void ControlNode::timer_callback() {
//   // RCLCPP_INFO(this->get_logger(), "Hello World...");

//   auto msg = std_msgs::msg::String();
//   msg.data = "Hello example_string topic...";
//   msg_pub_->publish(msg);
// }

// void ControlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//   auto xpos = msg->pose.pose.position.x;
//   auto ypos = msg->pose.pose.position.y;
//   auto zpos = msg->pose.pose.position.z;

// RCLCPP_INFO(this->get_logger(), "Goal co-ords: [x,y,z] = [%f, %f, %f]", xpos, ypos, zpos)
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
