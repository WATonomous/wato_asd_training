#include "planner_node.hpp"

// Implementation of CellIndex constructors
CellIndex::CellIndex(int xx, int yy) : x(xx), y(yy) {}
CellIndex::CellIndex() : x(0), y(0) {}

// Implementation of CellIndex equality operator
bool CellIndex::operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
}

// Implementation of CellIndex inequality operator
bool CellIndex::operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
}

// Implementation of CellIndexHash
std::size_t CellIndexHash::operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
}

// Implementation of CompareF
bool CompareF::operator()(const AStarNode &a, const AStarNode &b) {
    return a.f_score > b.f_score;
}

PlannerNode::PlannerNode()
    : Node("planner"), current_state_(PlannerState::WAITING_FOR_GOAL) {
    RCLCPP_INFO(this->get_logger(), "Initializing Planner Node");

    // Create publishers and subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    goal_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PlannerNode::handleState, this));

    last_progress_time_ = this->now();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    global_map_ = *map;

    if (current_state_ == PlannerState::WAITING_TO_REACH_GOAL) {
        RCLCPP_INFO(this->get_logger(), "Map updated, replanning...");
        planPath();
        publishPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
    current_goal_ = *goal;
    RCLCPP_INFO(this->get_logger(), "Received goal: (%f, %f)", goal->point.x, goal->point.y);

    current_state_ = PlannerState::WAITING_TO_REACH_GOAL;
    last_progress_time_ = this->now();
    last_robot_pose_ = current_odom_.pose.pose;

    planPath();
    publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    current_odom_ = *odom;
}

void PlannerNode::handleState() {
    if (current_state_ == PlannerState::WAITING_TO_REACH_GOAL) {
        double distance = std::sqrt(
            std::pow(current_odom_.pose.pose.position.x - current_goal_.point.x, 2) +
            std::pow(current_odom_.pose.pose.position.y - current_goal_.point.y, 2));

        // Check if the robot has reached the goal
        if (distance < 0.5) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            current_state_ = PlannerState::WAITING_FOR_GOAL;
            return;
        }

        // Check if the robot has made progress
        double progress = std::sqrt(
            std::pow(current_odom_.pose.pose.position.x - last_robot_pose_.position.x, 2) +
            std::pow(current_odom_.pose.pose.position.y - last_robot_pose_.position.y, 2));

        if (progress > 0.1) {  // Progress threshold
            last_progress_time_ = this->now();
            last_robot_pose_ = current_odom_.pose.pose;
        } else if ((this->now() - last_progress_time_).seconds() > 10.0) {  // Timeout duration
            RCLCPP_WARN(this->get_logger(), "Timeout reached, replanning...");
            planPath();
            last_progress_time_ = this->now();
            publishPath();
        }
    }
}

void PlannerNode::planPath() {
    path_.poses.clear();
    path_.header.frame_id = "sim_world";  // Ensure frame is "sim_world"
    path_.header.stamp = this->now();

    int width = global_map_.info.width;
    int height = global_map_.info.height;
    double resolution = global_map_.info.resolution;

    std::vector<std::vector<int>> grid(height, std::vector<int>(width, 0));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            grid[y][x] = (global_map_.data[index] == 0) ? 1 : 0;
        }
    }

    int start_x = static_cast<int>((current_odom_.pose.pose.position.x - global_map_.info.origin.position.x) / resolution);
    int start_y = static_cast<int>((current_odom_.pose.pose.position.y - global_map_.info.origin.position.y) / resolution);
    int goal_x = static_cast<int>((current_goal_.point.x - global_map_.info.origin.position.x) / resolution);
    int goal_y = static_cast<int>((current_goal_.point.y - global_map_.info.origin.position.y) / resolution);

    CellIndex src(start_y, start_x);
    CellIndex dest(goal_y, goal_x);

    if (!isValid(start_y, start_x) || !isValid(goal_y, goal_x)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid start or goal position");
        return;
    }

    if (!isUnBlocked(grid, start_y, start_x) || !isUnBlocked(grid, goal_y, goal_x)) {
        RCLCPP_ERROR(this->get_logger(), "Start or goal position is blocked");
        return;
    }

    if (isDestination(start_y, start_x, dest)) {
        RCLCPP_INFO(this->get_logger(), "Already at the goal");
        return;
    }

    std::vector<std::pair<int, int>> path = aStarSearch(grid, src, dest);

    path_.poses.clear();
    for (const auto &point : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "sim_world";  // Ensure frame is "sim_world"
        pose.pose.position.x = global_map_.info.origin.position.x + point.second * resolution;
        pose.pose.position.y = global_map_.info.origin.position.y + point.first * resolution;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_.poses.push_back(pose);
    }

    RCLCPP_INFO(this->get_logger(), "Path planned successfully");
}

std::vector<std::pair<int, int>> PlannerNode::aStarSearch(const std::vector<std::vector<int>> &grid,
                                                          const CellIndex &src, const CellIndex &dest) {
    if (!isValid(src.x, src.y) || !isValid(dest.x, dest.y)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid source or destination");
        return {};
    }

    if (!isUnBlocked(grid, src.x, src.y) || !isUnBlocked(grid, dest.x, dest.y)) {
        RCLCPP_ERROR(this->get_logger(), "Source or destination is blocked");
        return {};
    }

    if (isDestination(src.x, src.y, dest)) {
        RCLCPP_INFO(this->get_logger(), "Already at the destination");
        return {std::make_pair(src.x, src.y)};
    }

    std::unordered_map<CellIndex, bool, CellIndexHash> closedList;
    std::unordered_map<CellIndex, AStarNode, CellIndexHash> cellDetails;

    AStarNode startNode(src, 0.0);
    cellDetails[src] = startNode;

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> openList;
    openList.push(startNode);

    int rowNum[] = {-1, 1, 0, 0, -1, -1, 1, 1};
    int colNum[] = {0, 0, -1, 1, -1, 1, -1, 1};

    while (!openList.empty()) {
        AStarNode currentNode = openList.top();
        openList.pop();

        CellIndex currentIndex = currentNode.index;
        closedList[currentIndex] = true;

        for (int dir = 0; dir < 8; dir++) {
            int new_x = currentIndex.x + rowNum[dir];
            int new_y = currentIndex.y + colNum[dir];
            CellIndex neighbor(new_x, new_y);

            if (isValid(new_x, new_y)) {
                if (isDestination(new_x, new_y, dest)) {
                    cellDetails[neighbor] = AStarNode(currentIndex, 0.0);

                    std::vector<std::pair<int, int>> path;
                    CellIndex traceIndex = dest;
                    while (!(traceIndex == src)) {
                        path.push_back(std::make_pair(traceIndex.x, traceIndex.y));
                        traceIndex = cellDetails[traceIndex].index;
                    }
                    path.push_back(std::make_pair(src.x, src.y));
                    std::reverse(path.begin(), path.end());
                    return path;
                } else if (!closedList[neighbor] && isUnBlocked(grid, new_x, new_y)) {
                    double gNew = cellDetails[currentIndex].f_score + 1.0;
                    double hNew = calcHeuristic(new_x, new_y, dest);
                    double fNew = gNew + hNew;

                    if (cellDetails.find(neighbor) == cellDetails.end() || cellDetails[neighbor].f_score > fNew) {
                        cellDetails[neighbor] = AStarNode(currentIndex, fNew);
                        openList.push(AStarNode(neighbor, fNew));
                    }
                }
            }
        }
    }

    RCLCPP_ERROR(this->get_logger(), "Failed to find the destination cell");
    return {};
}

bool PlannerNode::isValid(int row, int col) {
    return (row >= 0) && (row < static_cast<int>(global_map_.info.height)) &&
           (col >= 0) && (col < static_cast<int>(global_map_.info.width));
}

bool PlannerNode::isUnBlocked(const std::vector<std::vector<int>> &grid, int row, int col) {
    return grid[row][col] == 1;
}

bool PlannerNode::isDestination(int row, int col, const CellIndex &dest) {
    return row == dest.x && col == dest.y;
}

double PlannerNode::calcHeuristic(int row, int col, const CellIndex &dest) {
    return std::sqrt((row - dest.x) * (row - dest.x) +
                     (col - dest.y) * (col - dest.y));
}

void PlannerNode::publishPath() {
    path_.header.stamp = this->now();
    path_.header.frame_id = "sim_world";  // Ensure frame is "sim_world"
    path_pub_->publish(path_);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
