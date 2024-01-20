#include <memory>
#include "control_node.hpp"
ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));
  
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  string_publisher = this->create_publisher<std_msgs::msg::String>("/example_string",20);

  // subscriber_= this->create_subscription<nav_msgs::msg::Odometry>(
  //   "/model/robot/odometry", 20, 
  // std::bind(&ControlNode::subscription_callback, this, 
  // std::placeholders::_1));

  geo_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point",20,
  std::bind(&ControlNode::geo_subscription_callback, this,
  std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void ControlNode::timer_callback(){
  if(new_goal_point_ == nullptr) return;

  geometry_msgs::msg::TransformStamped transform;


  try {
      transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
      auto transformed_point = geometry_msgs::msg::PointStamped();
      tf2::doTransform(*new_goal_point_, transformed_point, transform);

      RCLCPP_INFO(this->get_logger(), "x pos: %f", transformed_point.point.x);
      RCLCPP_INFO(this->get_logger(), "y pos: %f", transformed_point.point.y);
      RCLCPP_INFO(this->get_logger(), "z pos: %f", transformed_point.point.z);


      double x_val = Kp_linear  * transformed_point.point.x;
      double y_val = Kp_angular * transformed_point.point.z; 

      auto twist_message = geometry_msgs::msg::Twist();
      twist_message.linear.x = x_val;
      twist_message.angular.z = y_val;
      publisher_->publish(twist_message);
  } 
  
  
  catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }
  RCLCPP_INFO(this->get_logger(), "timer called");

  auto msg = std_msgs::msg::String();
  
  msg.data = "publish msg";
  
  RCLCPP_INFO(this->get_logger(), "calling timer");
  string_publisher->publish(msg);

}

// void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
// {
// 	// Print to the console
//   RCLCPP_INFO(this->get_logger(), "x position: %f", msg->pose.pose.position.x);
//   RCLCPP_INFO(this->get_logger(), "y position: %f", msg->pose.pose.position.y);
//   RCLCPP_INFO(this->get_logger(), "z position: %f", msg->pose.pose.position.z);
// 	// Publish a message
// }

void ControlNode::geo_subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
  
  new_goal_point_ = msg;


}




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
