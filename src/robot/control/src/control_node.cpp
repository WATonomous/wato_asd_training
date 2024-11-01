#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  goal_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::goal_point_subscription_callback, this, std::placeholders::_1));
  cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  pid_loop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::pid_loop_callback, this));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ControlNode::goal_point_subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "goal point (x,y,z): %f, %f, %f", msg->point.x, msg->point.y, msg->point.z);
  this->goal_point_ = *msg;
}

void ControlNode::pid_loop_callback() {
  // Transform goal point into robot-space before storing
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(this->goal_point_, transformed_point, transform);

  double lin_ctrl_signal{Kp_linear * transformed_point.point.x};
  double ang_ctrl_signal{Kp_angular * transformed_point.point.y};

  auto msg = geometry_msgs::msg::Twist();
  auto linear_component = geometry_msgs::msg::Vector3();
  auto angular_component = geometry_msgs::msg::Vector3();
  linear_component.set__x(lin_ctrl_signal);
  angular_component.set__z(ang_ctrl_signal);

  msg.set__linear(linear_component);
  msg.set__angular(angular_component);

  cmd_vel_publisher->publish(msg);

  RCLCPP_INFO(this->get_logger(), "linear/radial mtr outputs: %f, %f", lin_ctrl_signal, ang_ctrl_signal);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
