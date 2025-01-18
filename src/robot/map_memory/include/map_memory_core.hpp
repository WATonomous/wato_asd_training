#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
	nav_msgs::msg::OccupancyGrid::SharedPtr integrateCostmap(
		nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap);

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;

	bool is_global_map_init_ = false;
};

}  

#endif  
