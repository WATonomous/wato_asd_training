#ifndef NAV_NODE_HPP_
#define NAV_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class NavNode : public rclcpp::Node {
  public:
    NavNode();

  private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr next_point_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    robot::NavCore nav_;

    geometry_msgs::msg::PointStamped goal_point;

    void goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void occupancy_callback(nav_msgs::msg::OccupancyGrid::SharedPtr occupancy);
};

#endif
