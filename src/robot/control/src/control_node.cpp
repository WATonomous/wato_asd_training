#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "control_node.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 20,
    std::bind(&ControlNode::sub_callback, this,
    std::placeholders::_1)
  );

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
  std::bind(&ControlNode::timer_callback, this));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void ControlNode::sub_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(),
  "\n\nX = %f \nY = %f \nZ = %f \n\n",
  msg->point.x, msg->point.y, msg->point.z);

  original_point.point = msg->point;
}

void ControlNode::timer_callback() {

  try {
    this->transform = tf_buffer_->lookupTransform("robot", "sim_world",
    tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s",
    ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();

  tf2::doTransform(this->original_point, transformed_point, this->transform);

  double Kp_linear = 0.5;
  double Kp_angular = 0.5;
  Kp_linear = Kp_linear * transformed_point.point.x;
  Kp_angular = Kp_angular * transformed_point.point.y;

  twist_msg = geometry_msgs::msg::Twist();
  this->twist_msg.linear.x = Kp_linear;
  this->twist_msg.angular.z = Kp_angular;

  publisher_->publish(twist_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
