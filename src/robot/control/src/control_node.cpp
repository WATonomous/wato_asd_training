#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  //create the timer first 
  timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&ControlNode::timer_callback, this));

  //6.2 - timer that calls blank callback every 100 ms for control, print some random message
  goal_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::goal_timer_callback, this));

  //6.2 transforms define the buffer and listenenr
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  //create the publisher 
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  final_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  //create the subscriber 
   // Create the subscriber
  subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/model/robot/odometry", 10,
      std::bind(&ControlNode::odometry_callback, this, std::placeholders::_1));
  
  //deliverable 6.1: Create a subscriber that receives geometry_msgs::msg::PointStamped messages from the “/goal_point” topic.
  goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&ControlNode::goal_point_callback, this, std::placeholders::_1));
  
  
}

void ControlNode::timer_callback(){
  //Print out the message of your choice 
  RCLCPP_INFO(this->get_logger(), "Hello World!");

  //Create and populate Twist - velocity in free space linear and angular 
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = 0.5;
  message.angular.z = 2;

  //publish the message
  publisher_->publish(message);

}

//timer callback function (6.2)
void ControlNode::goal_timer_callback(){
  //Print out the message of your choice 
  RCLCPP_INFO(this->get_logger(), "Hello World from Goal Timer!");

  // Ensure goal_robot_point_ is initialized
  if (!goal_robot_point_) {
      RCLCPP_WARN(this->get_logger(), "Goal point not received yet.");
      return;
  }

    
    

    RCLCPP_INFO(this->get_logger(), "Control Signals - Linear Velocity: %.2f, Angular Velocity: %.2f", 
                linear_velocity, angular_velocity);
  //perform the transform from sim_world into robot 
  geometry_msgs::msg::TransformStamped transform;

  try{
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  }
  catch(const tf2::TransformException &ex){
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }

  //transform the point
  auto transformed_point = geometry_msgs::msg::PointStamped();
  
  //set the new transformed point now 
  try{
    tf2::doTransform(*goal_robot_point_, transformed_point, transform);

    RCLCPP_INFO(this->get_logger(), "Transformed Point - x: %.2f, y: %.2f", transformed_point.point.x, transformed_point.point.y);

    // Calculate control signals
    linear_velocity = Kp_linear * transformed_point.point.x;
    angular_velocity = Kp_angular * transformed_point.point.y;
  }catch(const tf2::TransformException &ex){
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }
  //Create and populate Twist - velocity in free space linear and angular 
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = linear_velocity;
  message.angular.z = angular_velocity;

  //publish the message
  final_publisher_->publish(message);


}

//odometry callback function 
void ControlNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract the x, y, z position from the Odometry message
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;

  // Print the coordinates to the console
  RCLCPP_INFO(this->get_logger(), "Odometry Position - x: %.2f, y: %.2f, z: %.2f", x, y, z);
}

//goal point callback function (6.2)
void ControlNode::goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
 
  //store the points in an instance of goal_robot_point_
  goal_robot_point_ = std::make_shared<geometry_msgs::msg::PointStamped>(*msg);

   // Extract the x, y, z position from the PointStamped message
  double x = goal_robot_point_->point.x;
  double y = goal_robot_point_->point.y;
  double z = goal_robot_point_->point.z;

  // Print the coordinates to the console
  RCLCPP_INFO(this->get_logger(), "Goal Point - x: %.2f, y: %.2f, z: %.2f", x, y, z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
