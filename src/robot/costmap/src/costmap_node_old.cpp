#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())), resolution_(0.1), 
  map_size_(15), origin_index_(map_size_ / resolution_), num_rows_(2 * map_size_ / resolution_), num_cols_(2 * map_size_ / resolution_), 
  matrix(num_rows_, std::vector<int>(num_cols_, 0)) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, 
    std::bind(
      &CostmapNode::lidar_callback, this,
      std::placeholders::_1));
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
}
 
void CostmapNode::publishMessage() {
  // auto message = std_msgs::msg::String();
  // message.data = "Hello, ROS 2!";
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  // string_pub_->publish(message);

  auto message = nav_msgs::msg::OccupancyGrid();
  message.header.stamp = this->now();
  message.header.frame_id = "map";
  
  message.info.resolution = resolution_;
  message.info.width = num_cols_;
  message.info.height = num_rows_;
  message.info.origin.position.x = -map_size_;
  message.info.origin.position.y = -map_size_;
  
  message.data.resize(num_rows_ * num_cols_);
  for (int i = 0; i < num_rows_; ++i) {
      for (int j = 0; j < num_cols_; ++j) {
          message.data[i * num_cols_ + j] = matrix[i][j] * 100; // Convert to occupancy values (0-100)
      }
  }
  
  costmap_pub_->publish(message);
}

// void inflateObstacles() {
//   for (int i = 0; i < num_rows_) {
//     for (int j = 0; j < num_cols_) {
//       if (matrix[i][j] == 1) {
//         for (int u = 0; )
//       }
//     }
//   }
  
// }

void CostmapNode::lidar_callback(sensor_msgs::msg::LaserScan msg) 
{
  RCLCPP_INFO(this->get_logger(), "lidar msg - angle_min: '%f', angle_max: '%f', angle_increment: %f, range_min: %f, range_max: %f", 
    msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min, msg.range_max);
  float angle = msg.angle_min;
  float angle_increment = msg.angle_increment;
  for (auto r : msg.ranges) {
    if (!std::isfinite(r)) {
      continue;
    }
    auto grid_indices = compute_grid_indices(r, angle);

    if (grid_indices.first >= 0 && grid_indices.first < num_rows_ && 
      grid_indices.second >= 0 && grid_indices.second < num_cols_) {
      matrix[grid_indices.first][grid_indices.second] = 1;
    } else {
      RCLCPP_ERROR(this->get_logger(), "out of bounds. row: %d, col: %d, range: %f, ang: %f", grid_indices.first, grid_indices.second, r, angle);
    }
    angle += angle_increment;
  }
  publishMessage();
  // RCLCPP_INFO(this->get_logger(), "count: %d", count);

}

std::pair<int, int> CostmapNode::compute_grid_indices(float range, float angle) {
  float x = range * cos(angle);
  float y = range * sin(angle);
  int row, col;
  if (x == map_size_) {
    col = num_cols_;
  } else {
    col = x / resolution_ + origin_index_;
  }

  if (y == map_size_) {
    row = num_rows_;
  } else {
    row = y / resolution_ + origin_index_;
  }

  return {row, col};
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}