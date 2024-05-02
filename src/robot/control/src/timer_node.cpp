#include "timer_node.hpp"

TimerNode::TimerNode() : Node("timer_node"), message_count_(0)
{
    string_publisher_=this->create_publisher<std_msgs::msg::String>("/example_string",20);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TimerNode::timer_callback, this));
    //Odometry subscriber
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/model/robot/odometry", 20,
        std::bind(&TimerNode::odometry_callback, this, std::placeholders::_1));
}

void TimerNode::timer_callback()
{
    message_count_++;
    // Create the message
    std_msgs::msg::String msg;
    msg.data = "Hello there. This is message number " + std::to_string(message_count_);

    // Publish the message
    string_publisher_->publish(msg);


    RCLCPP_INFO(this->get_logger(), "Message %zu: Hello there.", message_count_);
}

void TimerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto position = msg->pose.pose.position;
    RCLCPP_INFO(this->get_logger(), "Received odometry: x=%f, y=%f, z=%f", position.x, position.y, position.z);
}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TimerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
