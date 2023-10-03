#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    //5.1
    rclcpp::TimerBase::SharedPtr timer_1;
    //5.2
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    //5.3
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    //6.1
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pointstamped_subscriber_;

    //6.2
    rclcpp::TimerBase::SharedPtr timer_2; 
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::PointStamped goal_point_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

    double Kp_linear = 0.5;
    double Kp_angular = 0.5;
    double goal_point_transformed_x; 
    double goal_point_transformed_y;

    //5.1
    void timer_callback();
    //5.3
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    //6.1
    void pointstamped_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    //6.2
    void proportional_controller_callback(); 
};

#endif
