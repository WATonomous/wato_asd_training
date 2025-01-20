#include "planner_core.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger), OCCUPIED_THRESHOLD(50) {}

std::optional<nav_msgs::msg::Path> PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Point& goal) {
        
    auto [start_idx, goal_idx] = world_to_grid(start, goal, map);

    // Check if start or goal is invalid
    if (!is_valid_cell(start_idx, map) || !is_valid_cell(goal_idx, map)) {
        // RCLCPP_INFO(logger_, "Invalid or occupied start or goal pos");
        return std::nullopt;
    }

    // open set (heap)
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    open_set.push(AStarNode(start_idx, 0.0));

    // came_from unordered map
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    g_score[start_idx] = 0.0;

    std::unordered_map<CellIndex, double, CellIndexHash> f_score;
    f_score[start_idx] = h_score(start_idx, goal_idx);

    while (!open_set.empty()) {
        CellIndex current = open_set.top().index;
        open_set.pop();

        if (current == goal_idx) {
            return reconstruct_path(came_from, current, map);
        }

        auto neighbours = get_neighbours(current, map);
        for (const auto& neighbour : neighbours) {
            double tentative_g_score = g_score[current] + 
                (current.x != neighbour.x && current.y != neighbour.y ? 14 : 10);

            // If this path is better than previous one
            if (!g_score.count(neighbour) || tentative_g_score < g_score[neighbour]) {
                came_from[neighbour] = current;
                g_score[neighbour] = tentative_g_score;
                f_score[neighbour] = tentative_g_score + h_score(neighbour, goal_idx);
                open_set.push(AStarNode(neighbour, f_score[neighbour]));
            }
        }
    }

    // no path found
    return std::nullopt;
}

std::vector<CellIndex> PlannerCore::get_neighbours(
    const CellIndex& cell,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {

    std::vector<CellIndex> neighbours;
    neighbours.reserve(8);

    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;

            CellIndex neighbour(cell.x + dx, cell.y + dy);
            if (is_valid_cell(neighbour, map)) {
                neighbours.push_back(neighbour);
            }
        }
    }

    return neighbours;
}

bool PlannerCore::is_valid_cell(
    const CellIndex& cell,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map) const {
        
    if (cell.x < 0 || cell.x >= static_cast<int>(map->info.width) ||
        cell.y < 0 || cell.y >= static_cast<int>(map->info.height)) {
        return false;
    }

    // Check if cell is not occupied
    int idx = cell.y * map->info.width + cell.x;
    return map->data[idx] < OCCUPIED_THRESHOLD;
}

double PlannerCore::h_score(const CellIndex& a, const CellIndex& b) const {
    // Euclidean distance
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::pair<CellIndex, CellIndex> PlannerCore::world_to_grid(const geometry_msgs::msg::Pose& start, 
    const geometry_msgs::msg::Point& goal, const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {

    int start_x = static_cast<int>((start.position.x - map->info.origin.position.x) / map->info.resolution);
    int start_y = static_cast<int>((start.position.y - map->info.origin.position.y) / map->info.resolution);

    int goal_x = static_cast<int>((goal.x - map->info.origin.position.x) / map->info.resolution);
    int goal_y = static_cast<int>((goal.y - map->info.origin.position.y) / map->info.resolution);

    return {CellIndex(start_x, start_y), CellIndex(goal_x, goal_y)};
}

geometry_msgs::msg::Point PlannerCore::grid_to_world(const CellIndex& cell,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map) const {

    geometry_msgs::msg::Point point;
    point.x = cell.x * map->info.resolution + map->info.origin.position.x;
    point.y = cell.y * map->info.resolution + map->info.origin.position.y;
    point.z = 0.0;
    return point;
}

nav_msgs::msg::Path PlannerCore::reconstruct_path(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
    const CellIndex& current,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map) {

    nav_msgs::msg::Path path;
    path.header = map->header;
    
    // Start from the goal and work backwards
    std::vector<CellIndex> grid_path;
    CellIndex current_cell = current;
    
    while (came_from.count(current_cell)) {
        grid_path.push_back(current_cell);
        current_cell = came_from.at(current_cell);
    }
    grid_path.push_back(current_cell);
    
    // convert grid cells path to world poses
    for (auto it = grid_path.rbegin(); it != grid_path.rend(); ++it) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = map->header;
        pose.pose.position = grid_to_world(*it, map);
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    
    return path;
}

}