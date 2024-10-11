#include <memory>

#include "control_node.hpp"

using namespace std;

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore()) {
  
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  controlTimer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::control_timer_callback, this));
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  twistPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  subscriber_= this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  goalSubscriber = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::goalSubscription_callback, this, std::placeholders::_1));
  
  // For transform system -----------------------------------------------------------------
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // ----------------------------------------------------------------------------------------
}

void ControlNode::control_timer_callback() {

  // P-Controller Vars
  double Kp_linear = 0.9;
  double Kp_angular = 0.5;
  double x_velocity, y_velocity;

  try {
      transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } 
  
  catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  geometry_msgs::msg::PointStamped transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(original_point_, transformed_point, transform);

  x_velocity = Kp_linear*transformed_point.point.x;
  y_velocity = Kp_angular*transformed_point.point.y;

  // Creating Twist message
  geometry_msgs::msg::Twist twistMsg = geometry_msgs::msg::Twist();
  twistMsg.linear.x = x_velocity;
  twistMsg.angular.z = y_velocity;
  twistPublisher->publish(twistMsg);
  RCLCPP_INFO(this->get_logger(), "Publishing Twist message: linear.x = %f, angular.z = %f",
                twistMsg.linear.x, twistMsg.angular.z);

  // Testing transformation -------------------------------------------------------
  // double x = transformed_point.point.x;
  // double y = transformed_point.point.y;
  // RCLCPP_INFO(this->get_logger(), "Received message… x=%f, y=%f, x, y);
  // ------------------------------------------------------------------------------
}

void ControlNode::goalSubscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  original_point_ = *msg;
  
  double x = msg->point.x;
  double y = msg->point.y;
  double z = msg->point.z;

  RCLCPP_INFO(this->get_logger(), "Received message… x=%f, y=%f, z=%f", x, y, z);
}

void ControlNode::timer_callback() {

  // RCLCPP_INFO(this->get_logger(), "Hello World - Manjot");
  auto message = std_msgs::msg::String();
  message.data = "Hello World - Manjot!";
  publisher_->publish(message);
  
}

void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;

  //RCLCPP_INFO(this->get_logger(), "Received message… x=%f, y=%f, z=%f", x, y, z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
