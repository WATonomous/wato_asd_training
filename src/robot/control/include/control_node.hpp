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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/msg/string.h"
#include "nav_msgs/msg/odometry.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    void timer_callback();

    //this is for the /goal_point topic
    void goal_timer_callback();
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    //6.3 trasforms
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //define the instance of goal point so that you can collect this as the original robot coordinate
    std::shared_ptr<geometry_msgs::msg::PointStamped> goal_robot_point_;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr goal_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr final_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscriber_;

    double linear_velocity;
    double angular_velocity;
    double Kp_linear = 0.5;
    double Kp_angular = 0.5;

  private:
    robot::ControlCore control_;
};

#endif
