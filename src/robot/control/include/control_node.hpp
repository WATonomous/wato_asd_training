#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    //cl = current location; es = example string
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_cl_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_es_;

    //sl = selected location
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_sl_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_control_;

    //Timers
    rclcpp::TimerBase::SharedPtr timer_100_;
    rclcpp::TimerBase::SharedPtr timer_1000_;

    std::shared_ptr<geometry_msgs::msg::PointStamped> goal_point;
    
    robot::ControlCore control_;

    //Transformers
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    void timer_100_callback();
    void timer_1000_callback();

    void subscription_callback_current_location(const nav_msgs::msg::Odometry::SharedPtr msg);
    void subscription_callback_selected_location(const geometry_msgs::msg::PointStamped::SharedPtr msg);
};

#endif
