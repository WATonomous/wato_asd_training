#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), distance_threshold_(5) {
	costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, 
		std::bind(
			&MapMemoryNode::costmap_callback, this,
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, 
		std::bind(
			&MapMemoryNode::odom_callback, this,
			std::placeholders::_1));
	map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
	timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmap_callback(nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
	latest_costmap_ = costmap;
	costmap_updated_ = true;
}

void MapMemoryNode::odom_callback(nav_msgs::msg::Odometry::SharedPtr odom_msg) {
	double x = odom_msg->pose.pose.position.x;
	double y = odom_msg->pose.pose.position.y;

	// Compute distance traveled
	double distance = std::sqrt(std::pow(x - robot_x_, 2) + std::pow(y - robot_y_, 2));
	if (distance >= distance_threshold_) {
		robot_x_ = x;
		robot_y_ = y;
		robot_yaw_ = quaternionToYaw(odom_msg->pose.pose.orientation);
		should_update_map_ = true;
	}
}

double MapMemoryNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
	double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	return std::atan2(siny_cosp, cosy_cosp);
}

void MapMemoryNode::updateMap() {
	if (should_update_map_ && costmap_updated_) {
		nav_msgs::msg::OccupancyGrid::SharedPtr global_map = map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
		global_map->header.frame_id = "sim_world";
		map_pub_->publish(*global_map);
		should_update_map_ = false;
	}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
