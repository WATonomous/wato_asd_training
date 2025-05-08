#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <unordered_map>
#include <queue>
#include <vector>
#include <cmath>

// 2D grid index
struct CellIndex {
    int x;
    int y;

    CellIndex(int xx, int yy);
    CellIndex();

    bool operator==(const CellIndex &other) const;
    bool operator!=(const CellIndex &other) const;
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const;
};

// Structure representing a node in the A* open set
struct AStarNode {
    CellIndex index;
    double f_score;  // f = g + h

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
    AStarNode() : index(CellIndex()), f_score(0.0) {}  // Default constructor
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b);
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    // Enum for Planner State
    enum class PlannerState {
        WAITING_FOR_GOAL,
        WAITING_TO_REACH_GOAL
    };

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr goal_timer_;

    // Callback functions
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void handleState();

    // A* Search
    std::vector<std::pair<int, int>> aStarSearch(const std::vector<std::vector<int>> &grid,
                                                  const CellIndex &src, const CellIndex &dest);
    bool isValid(int row, int col);
    bool isUnBlocked(const std::vector<std::vector<int>> &grid, int row, int col);
    bool isDestination(int row, int col, const CellIndex &dest);
    double calcHeuristic(int row, int col, const CellIndex &dest);

    // Publisher function
    void publishPath();

    // Path planning
    void planPath();

    // Variables
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::PointStamped current_goal_;
    nav_msgs::msg::Path path_;

    // State
    PlannerState current_state_;

    // New variables
    rclcpp::Time last_progress_time_;  // Tracks the last time progress was made
    geometry_msgs::msg::Pose last_robot_pose_;  // Last known robot pose
};

#endif
