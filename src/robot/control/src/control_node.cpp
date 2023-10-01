#include <memory>
#include "control_node.hpp"



ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore()){
  goal_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::goalPointCallback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 160);
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::control_timer_callback, this));

  subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 160, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void ControlNode::timer_callback(){
  RCLCPP_INFO(this->get_logger(), "This is timer call back!");

  auto msg = std_msgs::msg::String();
  msg.data = "This is the Publish message";
  publisher_->publish(msg);
}

void ControlNode::goalPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  messages_object = *msg;
  RCLCPP_INFO(this->get_logger(), "Next Point: x=%f y=%f z=%f", messages_object.point.x, messages_object.point.y, messages_object.point.z);
  RCLCPP_INFO(this->get_logger(), "This is the blank timer callback");
}

void ControlNode::control_timer_callback(){
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();

  tf2::doTransform(messages_object, transformed_point , transform);

  RCLCPP_INFO(this->get_logger(), "Point: x=%f y=%f z=%f", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = Kp_linear * transformed_point.point.x;
  twist_msg.angular.z = Kp_angular * transformed_point.point.y;
  twist_publisher_->publish(twist_msg);

}



void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f",
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.position.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
