#ifndef NAV_NODE_HPP_
#define NAV_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_core.hpp"
//#include "nav_msgs/msg/odometry.hpp"


class NavNode : public rclcpp::Node {
  public:
    NavNode();

  private:

    // void NavNode::nav_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    robot::NavCore nav_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    
};

#endif
