#include "costmap_node.hpp"

CostmapNode::CostmapNode() 
    : Node("costmap_node"), width_(200), height_(200), resolution_(0.1),
      origin_x_(-10.0), origin_y_(-10.0), inflation_radius_(1.0) {
    
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    costmap_.resize(width_ * height_, 0);
    RCLCPP_INFO(this->get_logger(), "Costmap Node initialized");
}

void CostmapNode::initializeCostmap() {
    std::fill(costmap_.begin(), costmap_.end(), 0);  // Reset the costmap to free space
}

void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    double x_world = range * cos(angle);
    double y_world = range * sin(angle);

    x_grid = static_cast<int>((x_world - origin_x_) / resolution_);
    y_grid = static_cast<int>((y_world - origin_y_) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
    if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
        int index = y_grid * width_ + x_grid;
        costmap_[index] = 100;  // Mark as an obstacle
    }
}

void CostmapNode::inflateObstacles() {
    std::vector<int8_t> inflated_costmap = costmap_;  // Copy the current costmap

    int cells_inflation_radius = static_cast<int>(inflation_radius_ / resolution_);
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int index = y * width_ + x;
            if (costmap_[index] == 100) {  // If this cell is an obstacle
                for (int dy = -cells_inflation_radius; dy <= cells_inflation_radius; ++dy) {
                    for (int dx = -cells_inflation_radius; dx <= cells_inflation_radius; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                            if (distance <= inflation_radius_) {
                                int neighbor_index = ny * width_ + nx;
                                inflated_costmap[neighbor_index] = static_cast<int8_t>(
                                    std::max(static_cast<int>(inflated_costmap[neighbor_index]), 
                                             static_cast<int>(100 * (1 - distance / inflation_radius_)))
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    costmap_ = inflated_costmap;  // Update the costmap
}

void CostmapNode::publishCostmap() {
    nav_msgs::msg::OccupancyGrid costmap_msg;
    costmap_msg.header.stamp = this->now();
    costmap_msg.header.frame_id = "map";

    costmap_msg.info.resolution = resolution_;
    costmap_msg.info.width = width_;
    costmap_msg.info.height = height_;
    costmap_msg.info.origin.position.x = origin_x_;
    costmap_msg.info.origin.position.y = origin_y_;
    costmap_msg.info.origin.position.z = 0.0;

    costmap_msg.data = costmap_;

    costmap_pub_->publish(costmap_msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    initializeCostmap();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range > scan->range_min && range < scan->range_max) {
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);
        }
    }

    inflateObstacles();
    publishCostmap();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
