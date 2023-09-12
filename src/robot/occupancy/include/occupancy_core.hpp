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
    // Constants used for base map size
    static constexpr float MAP_HEIGHT_BASE = 40, MAP_WIDTH_BASE = 40;
    
  public:
    /**
     * Occupancy Core Constructor
     * 
     * @param map_resolution the map resolution
    */
    explicit OccupancyCore(float map_resolution);

    // Retrieves the Map Metadata for the Occupancy Message
    const nav_msgs::msg::MapMetaData get_map_meta_data();

    /**
     * Retrieves row-major occupancy data derived from the laser scan
     * polar coordinates
     * 
     * @param laser_scan the laser scan message
    */
    const std::vector<int8_t, std::allocator<int8_t>> get_occupancy_data(
      sensor_msgs::msg::LaserScan::SharedPtr laser_scan
    );
  
  private:
    // Map Meta Data Structure
    nav_msgs::msg::MapMetaData map_meta_data_;
    // Row Major Data Structure
    std::vector<int8_t, std::allocator<int8_t>> row_major_data_; 
    // Map Attributes
    float map_res_, map_height_, map_width_;

    /**
     * Initialize the Map Meta Data based on the given map resolution
     * 
     * @param map_resolution the map resolution
    */
    void init_map_metadata_(float map_resolution);

    /**
     * Clear the Row-major data structure for next laser scan. (This 
     * is naive, actual occupancy grids remember the past grid)
    */
    void clear_row_major_();

    /**
     * Filter inf ranges
     * @param ranges the ranges from the laser scan
    */
    void filter_ranges_(std::vector<float, std::allocator<float>>& ranges);

    /**
     * Convert laser scan polar coordinates to rectangular coordinates. 
     * Round them to the nearest square on the occupancy grid.
     * 
     * @param angle_min the min angle of the laser scan
     * @param angle_i the angle increment between readings
     * @param ranges the ranges from the laser scan
    */
    void polar_to_row_major_(
      const double& angle_min,
      const double& angle_i,
      const std::vector<float, std::allocator<float>>& ranges
    );
};

}  

#endif  
