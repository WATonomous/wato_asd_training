#include <memory>
#include <chrono>
#include <iostream>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore())
{
  // publish to a topic
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  // subscribe to a topic
  subscriber_= this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 20, 
  std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1));

  // timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  goal_point_received_ = false;
}

// Subscription callback: gets called when a message from the subscription comes in
void ControlNode::subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received Goal Point Stamped: x:%f, y:%f, z:%f", msg->point.x, msg->point.y, msg->point.z);
  this->goal_point_ = *msg;
  goal_point_received_ = true;
}

void ControlNode::timer_callback() {
    if (!goal_point_received_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s", e.what());
        return;
    }

    geometry_msgs::msg::PointStamped transformed_point;
    tf2::doTransform(goal_point_, transformed_point, transform);

    // Calculate forward and side motions from transformed points
    float linear = Kp_linear * transformed_point.point.x;
    float angular = Kp_angular * transformed_point.point.y;

    // Log transformed point in robot frame
    RCLCPP_INFO(this->get_logger(), "Transformed Point: x=%.2f, y=%.2f, z=%.2f", 
                transformed_point.point.x, 
                transformed_point.point.y, 
                transformed_point.point.z);
    
    // Create a publisher that publishes geometry_msgs::msg::Twist messages to the “/cmd_vel” topic. In your control timer, create a new Twist message, set the linear x component to the result of the linear proportional loop and the angular z component to the result of the angular proportional loop. Then, your robot should drive towards the goal_point message when you publish in Foxglove!
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = transformed_point.point.x;
    msg.angular.z = transformed_point.point.y;
    publisher_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
