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
    static constexpr int MAP_HEIGHT = 20, MAP_WIDTH = 20;
    static constexpr float MAP_RES = 1;
    
  public:
    explicit OccupancyCore();

    const nav_msgs::msg::MapMetaData get_meta_map_data();

    const std::vector<int8_t, std::allocator<int8_t>> get_occupancy_data(
      sensor_msgs::msg::LaserScan::SharedPtr laser_scan
    );
  
  private:
    nav_msgs::msg::MapMetaData map_meta_data_;
    std::vector<int8_t, std::allocator<int8_t>> row_major_data_; 

    void set_time_();

    void init_map_metadata_();

    void clear_row_major_();

    void polar_to_row_major_(
      const double& angle_max,
      const double& angle_min,
      const double& angle_i,
      const std::vector<float, std::allocator<float>>& ranges
    );
};

}  

#endif  
