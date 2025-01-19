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
		nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap, double robot_x, double robot_y, double robot_yaw);

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;

	void initializeGlobalmap();
	int width_cells;
	int height_cells;
	double resolution;
	std::pair<double, double> origin;

	std::pair<int, int> transformToGlobalCoordinates(
		const nav_msgs::msg::OccupancyGrid::SharedPtr& local_costmap, int local_x, int local_y, double robot_x, double robot_y, double robot_yaw);
};

}  

#endif  
