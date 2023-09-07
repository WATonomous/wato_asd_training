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

    void polar_to_rect();

    void init_map_metadata();

    void set_time();

    const int* get_occupancy_data(sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  
  private:
    nav_msgs::msg::MapMetaData map_meta_data_;

    int row_major_data_[MAP_HEIGHT * MAP_WIDTH];

};

}  

#endif  
