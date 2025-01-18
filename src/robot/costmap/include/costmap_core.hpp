#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);
    void initializeCostmap();
    nav_msgs::msg::OccupancyGrid::SharedPtr computeCostMap(sensor_msgs::msg::LaserScan::SharedPtr scan);

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_data_;
    int width_cells;
    int height_cells;
    double resolution;
    std::pair<double, double> origin; // This is the real-world pose of the cell (0,0) in the map.
    

    std::pair<int, int> compute_grid_indices(double range, double angle);
    void markObstacle(int row, int col);
    void inflateObstacles();
    double inflation_radius;
    int inflation_radius_cells;
    int max_cost;

};

}  

#endif  