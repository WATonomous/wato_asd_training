#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger), 
    width_cells(50),  height_cells(50), resolution(0.3), 
    origin(std::make_pair(-1*width_cells/2*resolution, -1*height_cells/2*resolution)),
    inflation_radius(0.9), inflation_radius_cells(inflation_radius/resolution), max_cost(80) {
        
    grid_data_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    initializeCostmap();
}

void CostmapCore::initializeCostmap() {
    grid_data_->info.width = width_cells;
    grid_data_->info.height = height_cells;
    grid_data_->info.resolution = resolution;
    grid_data_->info.origin.position.x = origin.first;
    grid_data_->info.origin.position.y = origin.second;
    grid_data_->info.origin.orientation.x = 0.0;
    grid_data_->info.origin.orientation.y = 0.0;
    grid_data_->info.origin.orientation.z = 0.0;
    grid_data_->info.origin.orientation.w = 1;

    grid_data_->data.assign(width_cells * height_cells, 0);
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::computeCostMap(
    sensor_msgs::msg::LaserScan::SharedPtr scan) {
    std::fill(grid_data_->data.begin(), grid_data_->data.end(), 0);
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            // Calculate grid coordinates
            auto indices = compute_grid_indices(range, angle);
            markObstacle(indices.first, indices.second);
        }
    }
    
    inflateObstacles();
    return grid_data_;
}

void CostmapCore::markObstacle(int row, int col) {
    if (col >=0 && col < width_cells && row >= 0 && row < height_cells) {
        grid_data_->data[row * width_cells + col] = 100;
    }
}

std::pair<int, int> CostmapCore::compute_grid_indices(double range, double angle) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
    int col = static_cast<int>(std::floor((x - grid_data_->info.origin.position.x) / resolution));
    int row = static_cast<int>(std::floor((y - grid_data_->info.origin.position.y) / resolution));

    return {row, col};
}

void CostmapCore::inflateObstacles() {
    for (int x = 0; x < width_cells; x++) {
        for (int y = 0; y < height_cells; y++) {
            if (grid_data_->data[y * width_cells + x] == 100) {
                for (int dx = -1*inflation_radius_cells; dx <= inflation_radius_cells; dx++) {
                    for (int dy = -1*inflation_radius_cells; dy <= inflation_radius_cells; dy++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        
                        if (nx < 0 || nx >= width_cells || ny < 0 || ny >= height_cells) {
                            continue;
                        }

                        double distance = std::sqrt(dx*dx + dy*dy) * resolution;
                        
                        if (distance > inflation_radius) {
                            continue;
                        }

                        int cost = static_cast<int>(max_cost * (1.0 - distance/inflation_radius));
                        
                        int index = ny * width_cells + nx;
                        if (cost > grid_data_->data[index] && grid_data_->data[index] != 100) {
                            grid_data_->data[index] = cost;
                        }
                    }
                }
            }
        }
    }
}

}