#include <memory>
#include "control_node.hpp"


ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore()){
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));

  odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>
  ("/model/robot/odometry", 20, std::bind(&ControlNode::odometry_callback, this, std::placeholders::_1));

  goal_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>
  ("/goal_point", 20, std::bind(&ControlNode::goal_point_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  control_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  Kp_linear = 0.3;
  Kp_angular = 0.3;

  original_goal_point_.point.x = 0.0;
  original_goal_point_.point.y = 0.0;
  original_goal_point_.point.z = 0.0;

  transformed_goal_point_.point.x = 0.0;
  transformed_goal_point_.point.y = 0.0;
  transformed_goal_point_.point.z = 0.0;
}

void ControlNode::timer_callback(){
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entered Timer Callback");

    // Transform goal point from sim_world to robot world
  geometry_msgs::msg::TransformStamped transform;

  if (goal_point_received){
    try {
        // Get the transform from "sim_world" to "robot"
        transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
        
        // Transform the goal_point to the robot's frame
        tf2::doTransform(original_goal_point_, transformed_goal_point_, transform);
        
        // Print the transformed coordinates
        RCLCPP_INFO(this->get_logger(), "Transformed goal_point - x: %f, y: %f, z: %f",
                    transformed_goal_point_.point.x,
                    transformed_goal_point_.point.y,
                    transformed_goal_point_.point.z);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform goal_point: %s", ex.what());
    }
  }

  double linear_control = Kp_linear * transformed_goal_point_.point.x;
  double angular_control = Kp_angular * transformed_goal_point_.point.y; 

  auto control_message = geometry_msgs::msg::Twist();
  control_message.linear.x = linear_control;
  control_message.angular.z = angular_control;
  control_publisher_->publish(control_message); 

}

void ControlNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract x, y, z coordinates from the odometry message
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;

  // Print the coordinates to the console
  RCLCPP_INFO(this->get_logger(), "Odometry coordinates - x: %f, y: %f, z: %f", x, y, z);
}

void ControlNode::goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  // Print Goal Point
  double x = msg->point.x;
  double y = msg->point.y;
  double z = msg->point.z;
  RCLCPP_INFO(this->get_logger(), "Received PointStamped coordinates - x: %f, y: %f, z: %f", x, y, z);

  original_goal_point_.point.x = msg->point.x;
  original_goal_point_.point.y = msg->point.y;
  original_goal_point_.point.z = msg->point.z;

  goal_point_received = true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

//rclcpp::get_logger("rclcpp")
//this->get_logger()

