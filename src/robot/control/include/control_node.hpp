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

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    // rclpp::Publisher<Unfiltered>::SharedPtr publisher_;
    rclpp::TimerBase::SharedPtr timer_;
    robot::ControlCore control_;

    void timer_callback();

};

#endif
