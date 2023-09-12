#include <algorithm>
#include <cmath>
#include <chrono>

#include "occupancy_core.hpp"

namespace robot
{

OccupancyCore::OccupancyCore(float map_resolution) 
  : row_major_data_(
    (MAP_HEIGHT_BASE / map_resolution) * (MAP_WIDTH_BASE / map_resolution), 0)
{
  // init map metadata
  init_map_metadata_(map_resolution);
}

void OccupancyCore::init_map_metadata_(float map_resolution) 
{
  map_res_ = map_resolution;
  map_height_ = MAP_HEIGHT_BASE / map_resolution;
  map_width_ = MAP_WIDTH_BASE / map_resolution;

  map_meta_data_.height = map_height_;
  map_meta_data_.width = map_width_;
  map_meta_data_.resolution = map_res_;

  map_meta_data_.origin.position.x = -map_width_ / 2 * map_res_;
  map_meta_data_.origin.position.y = -map_height_ / 2 * map_res_;
  map_meta_data_.origin.position.z = -0.9;
}

void OccupancyCore::clear_row_major_()
{
  for (auto& row_major_i : row_major_data_) {
    row_major_i = 0;
  }
}

void OccupancyCore::filter_ranges_(std::vector<float, std::allocator<float>>& ranges)
{
  for (auto& range : ranges) {
    if (std::isinf(range)) { range = -1; }
  }
}

const std::vector<int8_t, std::allocator<int8_t>> OccupancyCore::get_occupancy_data(
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan) 
{
  clear_row_major_();

  double angle_max = laser_scan->angle_max;
  double angle_min = laser_scan->angle_min;
  double angle_i = laser_scan->angle_increment;

  std::vector<float, std::allocator<float>> ranges = laser_scan->ranges;

  filter_ranges_(ranges);
  polar_to_row_major_(angle_max, angle_min, angle_i, ranges);

  return row_major_data_;
}

const nav_msgs::msg::MapMetaData OccupancyCore::get_meta_map_data() 
{
  return map_meta_data_;
}

void OccupancyCore::polar_to_row_major_(const double& angle_max, 
  const double& angle_min, const double& angle_i,
  const std::vector<float, std::allocator<float>>& ranges) 
{
  for (int i = 0; i < ranges.size(); i++) {
    if (ranges[i] < 0) { continue; }
    float cur_angle = angle_min + (angle_i * i);

    int x = std::round((ranges[i] * std::cos(cur_angle) / map_res_) + (map_width_ / 2));
    int y = std::round((ranges[i] * std::sin(cur_angle) / map_res_) + (map_height_ / 2));

    if (x + y * map_height_ < map_height_ * map_width_) {
      row_major_data_[x + y * map_height_] = 1;
    }
  }
}

} 
