#include <memory>
#include <cmath>
#include <algorithm>
#include <chrono>
  
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      last_x_(0.0), last_y_(0.0), distance_threshold_(1.5),
      current_x_(0.0), current_y_(0.0), current_theta_(0.0),
      map_width_(50), map_height_(50), map_resolution_(0.6) { // Set map memory attributes

    RCLCPP_INFO(this->get_logger(), "Initializing Map Memory Node");

    // Create publishers and subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));

    // Initialize the global map
    global_map_.header.frame_id = "sim_world";
    global_map_.info.width = map_width_;
    global_map_.info.height = map_height_;
    global_map_.info.resolution = map_resolution_;
    global_map_.info.origin.position.x = -15.0; // Center the map at (0, 0)
    global_map_.info.origin.position.y = -15.0;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.resize(map_width_ * map_height_, 0); // Initialize map data with zeros
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    if (costmap_update_counter_ < 5) {
        costmap_update_counter_++;
    }

    latest_costmap_ = *map;
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    // Extract position
    current_x_ = odom->pose.pose.position.x;
    current_y_ = odom->pose.pose.position.y;

    // Extract orientation and compute yaw directly
    double x = odom->pose.pose.orientation.x;
    double y = odom->pose.pose.orientation.y;
    double z = odom->pose.pose.orientation.z;
    double w = odom->pose.pose.orientation.w;

    // Compute yaw (rotation around z-axis)
    current_theta_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    // Calculate distance traveled
    double distance = std::sqrt(std::pow((current_x_ - last_x_), 2) + std::pow((current_y_ - last_y_), 2));
    if (distance >= distance_threshold_) {
        last_x_ = current_x_;
        last_y_ = current_y_;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap() {
    if (costmap_update_counter_ < 5) {
        RCLCPP_INFO(this->get_logger(), "Waiting for costmap to be updated");
        return;
    }

    if (!map_initialized_) {
        RCLCPP_INFO(this->get_logger(), "Map Memory not initialized yet");
        should_update_map_ = true;
        map_initialized_ = true;
    }
    if ((should_update_map_ && costmap_updated_)) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
        RCLCPP_INFO(this->get_logger(), "Map Memory updated");
    }
}

void MapMemoryNode::integrateCostmap() {
    int dim = latest_costmap_.info.width;
    double res = latest_costmap_.info.resolution;

    // Check if the costmap has a valid frame ID
    if (latest_costmap_.header.frame_id.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Costmap frame ID is empty!");
        return;
    }

    for (int local_grid_y = 0; local_grid_y < dim; ++local_grid_y) {
        for (int local_grid_x = 0; local_grid_x < dim; ++local_grid_x) {
            if (latest_costmap_.data[local_grid_y * dim + local_grid_x] != 0) {
                // Convert local grid coordinates to global coordinates
                double local_x = (local_grid_x - dim / 2) * res;
                double local_y = (local_grid_y - dim / 2) * res;

                // Transform local coordinates to the global frame (sim_world)
                double global_x = current_x_ + local_x * cos(current_theta_) - local_y * sin(current_theta_);
                double global_y = current_y_ + local_x * sin(current_theta_) + local_y * cos(current_theta_);

                // Convert global coordinates to global grid indices
                int global_grid_x = int(std::floor((global_x - global_map_.info.origin.position.x) / map_resolution_));
                int global_grid_y = int(std::floor((global_y - global_map_.info.origin.position.y) / map_resolution_));

                // Check bounds and update the global map
                if (global_grid_x >= 0 && global_grid_x < global_map_.info.width &&
                    global_grid_y >= 0 && global_grid_y < global_map_.info.height) {
                    global_map_.data[global_grid_y * global_map_.info.width + global_grid_x] = std::max(
                        global_map_.data[global_grid_y * global_map_.info.width + global_grid_x],
                        latest_costmap_.data[local_grid_y * dim + local_grid_x]);
                }
            }
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
