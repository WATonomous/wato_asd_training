#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <queue>
#include <unordered_map>
#include <optional>

namespace robot {

// 2D grid index
struct CellIndex {
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const {
        return (x == other.x && y == other.y);
    }

    bool operator!=(const CellIndex &other) const {
        return (x != other.x || y != other.y);
    }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

// A* node structure
struct AStarNode {
    CellIndex index;
    double f_score;  // f = g + h

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for priority queue
struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
        return a.f_score > b.f_score;
    }
};

class PlannerCore {
public:
    explicit PlannerCore(const rclcpp::Logger& logger);

	// May or may not return a path
    std::optional<nav_msgs::msg::Path> planPath(const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Point& goal);

private:
    rclcpp::Logger logger_;

    std::vector<CellIndex> get_neighbours(const CellIndex& cell, 
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map);
    
    bool is_valid_cell(const CellIndex& cell, 
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map) const;
    
    double h_score(const CellIndex& a, const CellIndex& b) const;
    
    std::pair<CellIndex, CellIndex> world_to_grid(
        const geometry_msgs::msg::Pose& start,
        const geometry_msgs::msg::Point& goal,
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map);
    
    geometry_msgs::msg::Point grid_to_world(
        const CellIndex& cell,
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map) const;
    
    nav_msgs::msg::Path reconstruct_path(
        const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
        const CellIndex& current,
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map);

    const int OCCUPIED_THRESHOLD;
};

}

#endif