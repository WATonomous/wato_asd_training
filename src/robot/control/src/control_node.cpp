#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_pose", 20, 
    std::bind(&ControlNode::goal_pose_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  using namespace std::chrono_literals;
  control_timer_ = this->create_wall_timer(1000ms, std::bind(&ControlNode::control_timer_callback, this));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  RCLCPP_INFO(this->get_logger(), "Control Node!");
}

void ControlNode::goal_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_pose = *msg;
  RCLCPP_INFO(this->get_logger(), "Goal Pose: x=%f y=%f z=%f", goal_pose.point.x, goal_pose.point.y, goal_pose.point.z);
}

void ControlNode::control_timer_callback(){
  geometry_msgs::msg::TransformStamped transform;
  // std::vector<std::string> frames;
  // tf_buffer_->_getFrameStrings(frames);

  // for (int i = 0; i < frames.size(); i++) {
  //     RCLCPP_INFO(this->get_logger(), "Transform: %s", frames[i].c_str());
  // }

  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  auto local_goal_pose = geometry_msgs::msg::PointStamped();

  tf2::doTransform(goal_pose, local_goal_pose, transform);

  RCLCPP_INFO(this->get_logger(), "Local Goal Pose: x=%f y=%f z=%f", local_goal_pose.point.x, local_goal_pose.point.y, local_goal_pose.point.z);

  float linear = control_.calculate_linear(local_goal_pose.point);
  float angular = control_.calculate_angular(local_goal_pose.point);

  RCLCPP_INFO(this->get_logger(), "linear=%f angular=%f", linear, angular);

  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = linear;
  twist.angular.z = angular;

  cmd_vel_pub_->publish(twist);


}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
