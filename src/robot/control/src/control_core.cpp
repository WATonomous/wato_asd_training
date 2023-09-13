#include <string>
#include <vector>

#include "control_core.hpp"

namespace robot
{
    ControlCore::ControlCore() {}

    float ControlCore::calculate_linear(geometry_msgs::msg::Point local_goal_pose){
        return std::min(1.0, std::max(this->kP_linear * local_goal_pose.x, -1.0));
    }

    float ControlCore::calculate_angular(geometry_msgs::msg::Point local_goal_pose){
        return std::min(1.0, std::max(this->kP_angular * local_goal_pose.y, -1.0));
    }
}  
