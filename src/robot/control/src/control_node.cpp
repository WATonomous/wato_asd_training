#include <memory>
#include "control_node.hpp"


ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore())
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
  
  // odometry_subscriber=this->create_subscription<nav_msgs::msg::Odometry>(
  //   "/model/robot/odometry",20,
  //   std::bind(&ControlNode::odometry_callback, this, std::placeholders::_1)
  // );

  // subscriber_ = this->create_subscription<std_msgs::msg::String>(
  //   "/example_string", 20,
  //   std::bind(&ControlNode::example_callback, this, std::placeholders::_1)
  // );
  
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::timer_callback, this));
  
  goal_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point",20,
    std::bind(&ControlNode::goal_point_callback, this, std::placeholders::_1)
  );

  goal_point_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",20);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  goal_point_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&ControlNode::goal_timer_callback,this));

  
}

// void ControlNode::timer_callback()
// {
//   // RCLCPP_INFO(this->get_logger(), "This is a timer callback");
//   // auto msg = std_msgs::msg::String();
//   // msg.data = "This is a published message";
//   // publisher_->publish(msg);
// }

void ControlNode::goal_timer_callback(){
    const double k_linear = 0.5;  
    const double k_angular = 0.1;

    double linear_velocity = k_linear * transformed_point.point.x;
    double angular_velocity = k_angular * transformed_point.point.y;
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_velocity;
    cmd_vel_msg.angular.z = angular_velocity;  

    // Publish the message
    goal_point_publisher_->publish(cmd_vel_msg);

    RCLCPP_INFO(this->get_logger(),"Linear Velocity: %f , Angular Velocitu: %f", linear_velocity,angular_velocity);
}

void ControlNode::goal_point_callback(geometry_msgs::msg::PointStamped::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "Received Point: x: %f, y: %f, z: %f", 
            msg->point.x, msg->point.y, msg->point.z);\

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  tf2::doTransform(*msg, transformed_point, transform);
}

// void ControlNode::example_callback(std_msgs::msg::String::SharedPtr msg)
// {
//   // RCLCPP_INFO(this->get_logger(), "This is a subscription callback: %s", msg->data.c_str());
// }

// // void ControlNode::odometry_callback(nav_msgs::msg::Odometry::SharedPtr msg){
// //   double x = msg->pose.pose.position.x;
// //   double y = msg->pose.pose.position.y;
// //   double z = msg->pose.pose.position.z;

// //   RCLCPP_INFO(this->get_logger(),"Received Odometry: x: %f, y: %f, z: %f", x, y, z);
// // }



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}