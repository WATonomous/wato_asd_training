#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  subscriber_= this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ControlNode::subscription_callback(geometry_msgs::msg::PointStamped::SharedPtr msg){
  auto position = *msg;
  auto position_x = msg->point.x;
  auto position_y = msg->point.y;
  auto position_z = msg->point.z;

  // Print the pose information
  RCLCPP_INFO(this->get_logger(), "Before Position (x, y, z): (%f, %f, %f)", position_x, position_y, position_z);
  geometry_msgs::msg::TransformStamped transform;

  try {
      transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(position, transformed_point, transform);
  transformed_point_ = transformed_point;

  auto new_position_x = transformed_point.point.x;
  auto new_position_y = transformed_point.point.y;
  auto new_position_z = transformed_point.point.z;

  RCLCPP_INFO(this->get_logger(), "After Position (x, y, z): (%f, %f, %f)", new_position_x, new_position_y, new_position_z);

  
}

void ControlNode::timer_callback(){
    //RCLCPP_INFO(this->get_logger(), "Hello World!");
    // Publish a string message
    // auto message = std_msgs::msg::String();
    // message.data = "Publishing a string message!";
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // publisher_->publish(message);
    auto message = geometry_msgs::msg::Twist();
    float kp_linear = 0.3;
    float kp_angular = 0.05; 
    auto goal_x = transformed_point_.point.x;
    auto goal_y = transformed_point_.point.y;
    float new_pos_x = goal_x * kp_linear;
    float new_pos_y = goal_y * kp_angular;
    message.linear.x = new_pos_x;
    message.angular.z = new_pos_y;
    publisher_->publish(message);

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}