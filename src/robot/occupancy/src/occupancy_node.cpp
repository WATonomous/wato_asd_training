#include <chrono>
#include <memory>

#include "occupancy_node.hpp"

OccupancyNode::OccupancyNode(float map_resolution) 
  : Node("occupancy"), occupancy_(robot::OccupancyCore(map_resolution))
{
  // Initialize ROS2 Constructs
  lidar_sub_ = 
  this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&OccupancyNode::laserscan_callback_, this, std::placeholders::_1)
  );

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/occupancy", 10
  );
}

void OccupancyNode::laserscan_callback_(
  const sensor_msgs::msg::LaserScan::SharedPtr msg) 
{
  nav_msgs::msg::OccupancyGrid occupancy_msg;

  // Populate Header
  occupancy_msg.header.stamp = get_time_();
  occupancy_msg.header.frame_id = "sim_world";

  // Retrieve Robot Transform
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform("sim_world", "robot/chassis/gpu_lidar", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }

  // Get occupancy data based on transform
  occupancy_msg.data = occupancy_.get_occupancy_data(msg, transform);
  // Get Map Meta Data
  occupancy_msg.info = occupancy_.get_map_meta_data();
  occupancy_msg.info.map_load_time = get_time_();

  // Publish
  occupancy_pub_->publish(occupancy_msg);
}

rclcpp::Time OccupancyNode::get_time_()
{
  return this->get_clock()->now();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // We set a map resolution of 0.5 here
  rclcpp::spin(std::make_shared<OccupancyNode>(0.5));
  rclcpp::shutdown();
  return 0;
}
