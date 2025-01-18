#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())), distance_threshold(5) {
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
	double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
	if (distance >= distance_threshold) {
		last_x = x;
		last_y = y;
		should_update_map_ = true;
	}
}

void MapMemoryNode::updateMap() {
	if (should_update_map_ && costmap_updated_) {
		nav_msgs::msg::OccupancyGrid::SharedPtr global_map = map_memory_.integrateCostmap(latest_costmap_);
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
