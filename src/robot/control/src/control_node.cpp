#include <memory>

#include "control_node.hpp"

#define timer_delay 100
#define Kp_linear 0.5
#define Kp_angular 0.5

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_delay), std::bind(&ControlNode::timer_callback, this));

  publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  subscriber_ = this -> create_subscription<geometry_msgs::msg::PointStamped>(
                        "/goal_point", 20, 
                        std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 
                        

}

void ControlNode::timer_callback(){
  float control_linear = Kp_linear * last_goal_point.point.x;
  float control_angular = Kp_angular * last_goal_point.point.y;

  auto twist_cmd = geometry_msgs::msg::Twist();
  twist_cmd.linear.x = control_linear;
  twist_cmd.angular.z = control_angular;
  publisher_ -> publish(twist_cmd);
  //RCLCPP_INFO(this->get_logger(), "Timer callback");
  // RCLCPP_INFO(this->get_logger(), "LAST GPOINT COORDS (x,y,z): (%f, %f, %f)", 
  //                 last_goal_point.point.x,
  //                 last_goal_point.point.y,
  //                 last_goal_point.point.z);
}

void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  
  geometry_msgs::msg::TransformStamped transform;
  auto transformed_point = geometry_msgs::msg::PointStamped();
  try {
      transform = tf_buffer_->lookupTransform("sim_world", "robot", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  tf2::doTransform(*msg, transformed_point, transform);
  float x = transformed_point.point.x;
  float y = transformed_point.point.y;
  float z = transformed_point.point.z;
  last_goal_point = transformed_point;

  RCLCPP_INFO(this->get_logger(), "GOAL POINT COORDS (x,y,z): (%f, %f, %f)", x,y,z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
