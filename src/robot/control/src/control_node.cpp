#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);
  subscriber_= this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, std::bind(&ControlNode::sub_callback, this, std::placeholders::_1));
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  kp_linear_ = 0.5;
  kp_angular_ = 0.5;
}

void ControlNode::timer_callback() {

  RCLCPP_INFO(this->get_logger(), "Timer Callback");
  if (goal_recieved_ ==  true) {


    geometry_msgs::msg::TransformStamped transform;
    auto transformed_point = geometry_msgs::msg::PointStamped();
  
  
    try {
        transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
        tf2::doTransform(goal_point_, transformed_point, transform);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
    }

    double error_x = kp_linear_ * transformed_point.point.x;
    double error_y = kp_angular_ * transformed_point.point.y;

    auto message = geometry_msgs::msg::Twist();
    message.linear.x = error_x;
    message.angular.z = error_y;

    publisher_->publish(message);

  }
}

void ControlNode::sub_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  auto x = msg->point.x;
  auto y = msg->point.y;
  auto z = msg->point.z;
  RCLCPP_INFO(this->get_logger(), "Received PointStamped: x=%.2f, y=%.2f, z=%.2f", x, y, z);

  goal_recieved_ = true;

  goal_point_ = *msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
