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

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:

    void sub_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void timer_callback();

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    geometry_msgs::msg::TransformStamped transform;
    geometry_msgs::msg::PointStamped original_point;
    geometry_msgs::msg::Twist twist_msg;

    robot::ControlCore control_;
};

#endif
