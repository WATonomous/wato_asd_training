#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode()
: Node("control"), control_(robot::ControlCore())
{
  // Transformers
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  new_goal_point = nullptr;

  // Publisher to "/example_string" topic
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  // Subscriber
  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point",
    20,
    std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));

  // Timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&ControlNode::timer_callback, this));
  RCLCPP_INFO(this->get_logger(), "Node Constructor");
}

void ControlNode::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "Timer callbacked!");

  // Verify that goal point has been set
  if (new_goal_point == nullptr) return;

  // Transforming
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
    auto goal_point = geometry_msgs::msg::PointStamped();
    tf2::doTransform(*new_goal_point, goal_point, transform);

    RCLCPP_INFO(this->get_logger(), "Transformed x_pos: %f", goal_point.point.x);
    RCLCPP_INFO(this->get_logger(), "Transformed y_pos: %f", goal_point.point.y);
    RCLCPP_INFO(this->get_logger(), "Transformed z_pos: %f", goal_point.point.z);

    // Control loop
    double x_output = Kp_linear * goal_point.point.x;
    double y_output = Kp_angular * goal_point.point.y;

    // Publish message
    RCLCPP_INFO(this->get_logger(), "Publishing to /cmd_vel: x: %f | y: %f", x_output, y_output);
    auto control_signal = geometry_msgs::msg::Twist();
    control_signal.linear.x = x_output;
    control_signal.angular.z = y_output;
    publisher_->publish(control_signal);

  } catch (...) {
    RCLCPP_INFO(this->get_logger(), "Could not transform");
  }
}

void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Message Received! x_pos: %f", msg->point.x);
  RCLCPP_INFO(this->get_logger(), "Message Received! y_pos: %f", msg->point.y);
  RCLCPP_INFO(this->get_logger(), "Message Received! z_pos: %f", msg->point.z);

  new_goal_point = msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
