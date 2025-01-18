#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {
	
	global_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::integrateCostmap(
	nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap) {
	
	if (!is_global_map_init_) {
		*global_map_ = *latest_costmap;
        is_global_map_init_ = true; 
	}
	
	return global_map_;

}

} 
