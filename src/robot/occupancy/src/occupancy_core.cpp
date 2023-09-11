#include <algorithm>
#include <cmath>
#include <chrono>

#include "occupancy_core.hpp"

namespace robot
{

OccupancyCore::OccupancyCore() 
  : row_major_data_(MAP_HEIGHT*MAP_WIDTH, 0)
{
  // init map metadata
  init_map_metadata_();
}

void OccupancyCore::init_map_metadata_() 
{
  map_meta_data_.height = MAP_HEIGHT;
  map_meta_data_.width = MAP_WIDTH;
  map_meta_data_.resolution = MAP_RES;
}

void OccupancyCore::clear_row_major_()
{
  for (auto& row_major_i : row_major_data_) {
    row_major_i = 0;
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

  polar_to_row_major_(angle_max, angle_min, angle_i, ranges);

  return row_major_data_;
}

const nav_msgs::msg::MapMetaData OccupancyCore::get_meta_map_data() 
{
  set_time_();
  return map_meta_data_;
}

void OccupancyCore::polar_to_row_major_(const double& angle_max, 
  const double& angle_min, const double& angle_i,
  const std::vector<float, std::allocator<float>>& ranges) 
{
  for (int i = 0; i < ranges.size(); i++) {
    int cur_angle = angle_min + angle_i * i;

    int x = ranges[i] * std::cos(cur_angle);
    int y = ranges[i] * std::sin(cur_angle);

    row_major_data_[x + y * MAP_HEIGHT] = 1;
  }
}

void OccupancyCore::set_time_() 
{
  int timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

  map_meta_data_.map_load_time.sec = timestamp / 1000;
  map_meta_data_.map_load_time.nanosec = (timestamp - (map_meta_data_.map_load_time.sec * 1000)) * 1000000;
}

} 
