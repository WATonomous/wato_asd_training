#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    bool in_grid(int grid_x, int grid_y);

    void initializeCostmap(double resolution, int width, int height, 
      const geometry_msgs::msg::Pose origin, double inflation_radius, int max_cost);
      
    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> publishCostmap() const;

  private:
    rclcpp::Logger logger_;

    // Costmap container
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> costmap_msg_;

    // Inflation parameters
    double inflation_radius_;
    int max_cost_;

    // Helper functions
    void convertToGrid(double x, double y, int& grid_x, int& grid_y);
    void markObstacle(int grid_x, int grid_y);
    void inflateObstacles(int grid_x, int grid_y);
};

}  

#endif  