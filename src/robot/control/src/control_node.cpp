#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  next_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/next_point", 20, 
    std::bind(&ControlNode::next_point_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  using namespace std::chrono_literals;
  control_timer_ = this->create_wall_timer(100ms, std::bind(&ControlNode::control_timer_callback, this));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  RCLCPP_INFO(this->get_logger(), "Control Node!");
}

void ControlNode::next_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  next_point = *msg;
  RCLCPP_INFO(this->get_logger(), "Next Point: x=%f y=%f z=%f", next_point.point.x, next_point.point.y, next_point.point.z);
}

void ControlNode::control_timer_callback(){
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto local_next_point = geometry_msgs::msg::PointStamped();

  tf2::doTransform(next_point, local_next_point, transform);

  RCLCPP_INFO(this->get_logger(), "Local Next Point: x=%f y=%f z=%f", local_next_point.point.x, local_next_point.point.y, local_next_point.point.z);

  float linear = control_.calculate_linear(local_next_point.point);
  float angular = control_.calculate_angular(local_next_point.point);

  RCLCPP_INFO(this->get_logger(), "linear=%f angular=%f", linear, angular);

  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = linear;
  twist.angular.z = angular;

  cmd_vel_pub_->publish(twist);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
