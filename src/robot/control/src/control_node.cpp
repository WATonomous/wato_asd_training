#include <chrono>
#include <functional>


#include "control_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore()){

  // subscriber to /goal_point topic
  subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 
    20, 
    std::bind(&ControlNode::subscription_callback, this, 
    std::placeholders::_1));

  //setup timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&ControlNode::timer_callback, 
    this));

  // Transform buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 
    20);
}

void ControlNode::timer_callback(){
  
    geometry_msgs::msg::TransformStamped transform;
  geometry_msgs::msg::PointStamped temp_point;

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
    tf2::doTransform(goal_point, temp_point, transform);
    RCLCPP_INFO(this->get_logger(), "Transform point is Position: [x: %.3f, y: %.3f, z: %.3f]",
                temp_point.point.x, 
                temp_point.point.y, 
                temp_point.point.z);
    //for pid control basics, error control
    double linear_control = kp_linear * temp_point.point.x;
    double angular_control = kp_angular * temp_point.point.y;

    geometry_msgs::msg::Twist motion_twist;
    motion_twist.linear.x = linear_control;
    motion_twist.angular.z = angular_control;
    
    publisher_->publish(motion_twist);


  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

}



void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped msg)
{
    // save message to the goal point member var
    goal_point = msg;

    RCLCPP_INFO(this->get_logger(), 
      "Goal Point is Position: [x: %.3f, y: %.3f, z: %.3f]",
                msg.point.x, 
                msg.point.y, 
                msg.point.z);
}



int main(int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
