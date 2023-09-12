#ifndef OCCUPANCY_CORE_HPP_
#define OCCUPANCY_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

/**
 * Implementation of the internal logic used by the Occupancy Node to
 * calculate the occupancy map.
 */
class OccupancyCore {
  public:
    static constexpr float MAP_HEIGHT_BASE = 40, MAP_WIDTH_BASE = 40;
    
  public:
    explicit OccupancyCore(float map_resolution);

    const nav_msgs::msg::MapMetaData get_meta_map_data();

    const std::vector<int8_t, std::allocator<int8_t>> get_occupancy_data(
      sensor_msgs::msg::LaserScan::SharedPtr laser_scan
    );
  
  private:
    nav_msgs::msg::MapMetaData map_meta_data_;
    std::vector<int8_t, std::allocator<int8_t>> row_major_data_; 
    float map_res_, map_height_, map_width_;

    void init_map_metadata_(float map_resolution);

    void clear_row_major_();

    void filter_ranges_(std::vector<float, std::allocator<float>>& ranges);

    void polar_to_row_major_(
      const double& angle_max,
      const double& angle_min,
      const double& angle_i,
      const std::vector<float, std::allocator<float>>& ranges
    );
};

}  

#endif  
