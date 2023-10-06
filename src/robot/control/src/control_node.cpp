#include <memory>
#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{

  //5.1 - 5.3 (commented out while working on deliverables 6.1-6.4)
    //timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&ControlNode::timer_callback, this));

    //publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);
    
    //subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/robot/odometry", 20, std::bind(&ControlNode::sub_callback, this, std::placeholders::_1));

    
    //6.1
    point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 20, std::bind(&ControlNode::point_callback, this, std::placeholders::_1));


    //6.2
    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&ControlNode::control_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    control_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
}


void ControlNode::sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "x = %f , y = %f , z = %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void ControlNode::timer_callback() {
  //Deliverable 5.1 code: RCLCPP_INFO(this->get_logger(), "Tron on top");
  auto msg = std_msgs::msg::String();
  msg.data = "Tron on top again";
  publisher_->publish(msg);
}


//Deliverable 6.1
void ControlNode::point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_point = *msg;
  RCLCPP_INFO(this->get_logger(), "Point coordinates in global frame: x = %f , y = %f , z = %f" , msg->point.x, msg->point.y, msg->point.z);
}


void ControlNode::control_callback() {
  geometry_msgs::msg::TransformStamped transform;

  try { 
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Transform failed %s", ex.what());
  }

  auto transformed_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(goal_point, transformed_point, transform);
  
  RCLCPP_INFO(this->get_logger(), "Transformed point coordinates (x=%f, y=%f, z=%f)", transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);

  auto control_signal = geometry_msgs::msg::Twist();
  control_signal.linear.x = transformed_point.point.x * Kp_linear;
  control_signal.angular.z = transformed_point.point.y * Kp_angular;

  control_publisher_->publish(control_signal);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
