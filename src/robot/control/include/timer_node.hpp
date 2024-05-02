#ifndef TIMER_NODE_HPP
#define TIMER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TimerNode : public rclcpp::Node
{
public:
    TimerNode();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    size_t message_count_;                     // To count the number of messages

};

#endif // TIMER_NODE_HPP
