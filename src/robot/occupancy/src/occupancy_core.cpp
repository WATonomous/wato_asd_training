#include <algorithm>
#include <chrono>

#include "occupancy_core.hpp"

namespace robot
{

OccupancyCore::OccupancyCore() {
  // init map metadata
  init_map_metadata();
}

void OccupancyCore::init_map_metadata() {
  map_meta_data_.height = MAP_HEIGHT;
  map_meta_data_.width = MAP_WIDTH;
  map_meta_data_.resolution = MAP_RES;
}

const int* OccupancyCore::get_occupancy_data(sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {

  return row_major_data_;
}

void OccupancyCore::set_time() {
  int timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

  map_meta_data_.map_load_time.sec = timestamp / 1000;
  map_meta_data_.map_load_time.nanosec = (timestamp - (map_meta_data_.map_load_time.sec * 1000)) * 1000000;
}

} 
