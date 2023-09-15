#ifndef NAV_CORE_HPP_
#define NAV_CORE_HPP_

#include <vector>
#include "AStar.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>

namespace robot
{

class NavCore {
  public:
    NavCore();

    std::vector<geometry_msgs::msg::Point> navigate(nav_msgs::msg::OccupancyGrid::SharedPtr occupancy, geometry_msgs::msg::Point local_goal_point);

};

} 

#endif 
