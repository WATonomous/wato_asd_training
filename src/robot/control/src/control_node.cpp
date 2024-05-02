#include <memory>
#include "control_node.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore())
{
    // Initialize the subscriber
    subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 20,
        std::bind(&ControlNode::subscriber_callback, this, std::placeholders::_1));

    // Initialize TF2 Buffer and Listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Creating the publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

    // Initialize the timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControlNode::timer_callback, this));
}

void ControlNode::subscriber_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    current_goal_ = *msg;
}

void ControlNode::timer_callback()
{
    // Function to be called every 100ms
    try {
        auto transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
        geometry_msgs::msg::PointStamped transformed_point;
        tf2::doTransform(current_goal_, transformed_point, transform);

        // Calculate control commands
        double linear_command = Kp_linear * transformed_point.point.x;
        double angular_command = Kp_angular * transformed_point.point.y;

        // Create and publish Twist message
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = linear_command;
        twist_msg.angular.z = angular_command;
        publisher_->publish(twist_msg);

        RCLCPP_INFO(this->get_logger(), "Control commands: Linear = %f, Angular = %f",
                    linear_command, angular_command);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
