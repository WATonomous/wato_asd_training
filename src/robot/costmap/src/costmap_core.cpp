#include "costmap_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : 
logger_(logger), costmap_msg_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {}

void CostmapCore::initializeCostmap(double resolution, int width, int height, 
    geometry_msgs::msg::Pose origin, double inflation_radius, int max_cost)
{
  costmap_msg_->info.resolution = resolution;
  costmap_msg_->info.width = width;
  costmap_msg_->info.height = height;
  costmap_msg_->info.origin = origin;
  costmap_msg_->data.assign(width * height, 0);
  inflation_radius_ = inflation_radius;
  max_cost_ = max_cost;

}

void CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan)
{
  std::fill(costmap_msg_->data.begin(), costmap_msg_->data.end(), 0);

  std::vector<std::pair<int, int>> obstacles;

  double angle = laser_scan->angle_min;
  
  for (int i = 0; i < static_cast<int>(laser_scan->ranges.size()); ++i) {
    angle += i * laser_scan->angle_increment;

    double range = laser_scan->ranges[i];
    if (range >= laser_scan->range_min && range <= laser_scan->range_max) {
        // Calculate grid coordinates
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);

        int grid_x, grid_y;
        convertToGrid(x, y, grid_x, grid_y);
        
        if(in_grid(grid_x, grid_y)) {
          markObstacle(grid_x, grid_y);
          obstacles.emplace_back(grid_x, grid_y);
        }
    }
  }

  for (const auto& [grid_x, grid_y] : obstacles) {
    inflateObstacles(grid_x, grid_y);
  }

}

void CostmapCore::convertToGrid(double x, double y, int& grid_x, int& grid_y)
{
  double origin_x = costmap_msg_->info.origin.position.x;
  double origin_y = costmap_msg_->info.origin.position.y;
  double resolution = costmap_msg_->info.resolution;

  grid_x = static_cast<int>((x - origin_x) / resolution);
  grid_y = static_cast<int>((y - origin_y) / resolution);
}

bool CostmapCore::in_grid(int grid_x, int grid_y)
{
  return (grid_x >= 0 && grid_x < static_cast<int>(costmap_msg_->info.width)) &&
          (grid_y >= 0 && grid_y < static_cast<int>(costmap_msg_->info.height));
}

void CostmapCore::markObstacle(int grid_x, int grid_y)
{
  int width_ = costmap_msg_->info.width;

  if (!in_grid(grid_x, grid_y)) return;

  int index = grid_y * width_ + grid_x;
  costmap_msg_->data[index] = max_cost_;
}

void CostmapCore::inflateObstacles(int grid_x, int grid_y)
{
  int width = costmap_msg_->info.width;
  int resolution = costmap_msg_->info.resolution;

  int inflation_cells = static_cast<int>(inflation_radius_ / resolution);

  for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      int inflated_x = grid_x + dx;
      int inflated_y = grid_y + dy;

      if (!in_grid(inflated_x, inflated_y)) continue;

      double distance = std::sqrt(dx * dx + dy * dy) * resolution;
      if (distance <= inflation_radius_) {
        int cost = static_cast<int>(max_cost_ * (1.0 - distance / inflation_radius_));
        int index = inflated_y * width + inflated_x;
        if (cost > costmap_msg_->data[index]) {
          costmap_msg_->data[index] = cost;
        }
      }
    }
  }
}

std::shared_ptr<nav_msgs::msg::OccupancyGrid> CostmapCore::publishCostmap() const {
    return costmap_msg_;
}


}