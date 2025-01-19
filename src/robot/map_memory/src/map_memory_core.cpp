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
    global_map_->info.origin.position.x = origin.first;
    global_map_->info.origin.position.y = origin.second;
    global_map_->info.origin.orientation.x = 0.0;
    global_map_->info.origin.orientation.y = 0.0;
    global_map_->info.origin.orientation.z = 0.0;
    global_map_->info.origin.orientation.w = 1;

    global_map_->data.assign(width_cells * height_cells, 0);
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::integrateCostmap(
	nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap, double robot_x, double robot_y, double robot_yaw) {
	
	// Iterate through each cell in the local costmap
	for (size_t local_y = 0; local_y < latest_costmap->info.height; ++local_y) {
		for (size_t local_x = 0; local_x < latest_costmap->info.width; ++local_x) {
			int local_index = local_y * latest_costmap->info.width + local_x;
			int cost = latest_costmap->data[local_index];
			
			// Skip if no obstacle
			if (cost == 0) {
				continue;
			}
			
			// Transform to global coordinates
			auto [global_x, global_y] = transformToGlobalCoordinates(latest_costmap, local_x, local_y, robot_x, robot_y, robot_yaw);
			
			// Check if within global map bounds
			if (global_x >= 0 && global_x < width_cells && 
				global_y >= 0 && global_y < height_cells) {
				
				int global_index = global_y * width_cells + global_x;
				
				if (global_map_->data[global_index] >= 0) {
					global_map_->data[global_index] = cost;
				}
			}
		}
	}
	
	return global_map_;
}

std::pair<int, int> MapMemoryCore::transformToGlobalCoordinates(
	const nav_msgs::msg::OccupancyGrid::SharedPtr& local_costmap, int local_x, int local_y,
	double robot_x, double robot_y, double robot_yaw) {

	// Calculate local robot position in meters
	double local_pos_x = local_x * local_costmap->info.resolution + local_costmap->info.origin.position.x;
	double local_pos_y = local_y * local_costmap->info.resolution + local_costmap->info.origin.position.y;
	
	// Apply rotation transformation
	double cos_yaw = std::cos(robot_yaw);
	double sin_yaw = std::sin(robot_yaw);
	
	// Transform to global coordinates using robot's position
	double global_pos_x = local_pos_x * cos_yaw - local_pos_y * sin_yaw + robot_x;
	double global_pos_y = local_pos_x * sin_yaw + local_pos_y * cos_yaw + robot_y;
	
	// Convert to global grid coordinates
	int global_x = static_cast<int>((global_pos_x - global_map_->info.origin.position.x) / global_map_->info.resolution);
	int global_y = static_cast<int>((global_pos_y - global_map_->info.origin.position.y) / global_map_->info.resolution);
	
	return {global_x, global_y};									
}

} 
