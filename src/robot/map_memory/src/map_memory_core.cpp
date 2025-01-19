#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger), width_cells(50),  height_cells(50), resolution(0.6), 
    origin(std::make_pair(-1*width_cells/2*resolution, -1*height_cells/2*resolution)) {
	
	global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
	initializeGlobalmap();
}

void MapMemoryCore::initializeGlobalmap() {
	global_map_->info.width = width_cells;
    global_map_->info.height = height_cells;
    global_map_->info.resolution = resolution;
    global_map_->info.origin.position.x = -1 * origin.first;
    global_map_->info.origin.position.y = -1 * origin.first;
    global_map_->info.origin.orientation.x = 0.0;
    global_map_->info.origin.orientation.y = 0.0;
    global_map_->info.origin.orientation.z = 0.0;
    global_map_->info.origin.orientation.w = 1;

    global_map_->data.assign(width_cells * height_cells, 0);
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::integrateCostmap(
	nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap, double robot_x, double robot_y, double robot_yaw) {
	
	
	
	return global_map_;

}

} 
