#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  // create subscriber
  subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
  "/goal_point", ADVERTISING_FREQ,
  std::bind(&ControlNode::subscription_callback, this,
  std::placeholders::_1));

  // create publisher
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", ADVERTISING_FREQ);

  // create timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
  bind(&ControlNode::timer_callback, this));

  // create transform buffer
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  geometry_msgs::msg::TransformStamped transform;
}

void ControlNode::subscription_callback(const 
geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // print to console
  auto pos_x = msg->point.x;
  auto pos_y = msg->point.y;
  auto pos_z = msg->point.z;
  RCLCPP_INFO(this->get_logger(), 
  "Position X:'%f', Position Y:'%f', Position Z: '%f'", 
  pos_x, pos_y, pos_z);

  // dereference and assign to instance variable
  goal_point_ = *msg;
}

void ControlNode::timer_callback(){
  // ensure null goal_point variable does not get transformed
  if (goal_point_.point.x == 0)
    return;

  // get current transform from robot current postion
  try {
    transform = tf_buffer_->lookupTransform("robot", "sim_world",
    tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }
  // transform goal point to robot reference frame
  auto new_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(goal_point_, new_point, transform);

  // create twist message to publish
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = Kp_linear * new_point.point.x;
  message.angular.z = Kp_angular * new_point.point.y;
  
  // publish message
  publisher_->publish(message);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
