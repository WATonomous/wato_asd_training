#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <vector>
#include "geometry_msgs/msg/point.hpp"
#include <algorithm>

namespace robot
{

class ControlCore {
  public:
    ControlCore();
    float calculate_linear(geometry_msgs::msg::Point local_goal_pose);
    float calculate_angular(geometry_msgs::msg::Point local_goal_pose);

  private:
    float kP_linear = 0.75;
    float kP_angular = 0.75;

};

} 

#endif 
