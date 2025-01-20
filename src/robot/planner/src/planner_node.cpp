#include "planner_node.hpp"
#include <functional>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::map_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goal_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odom_callback, this, std::placeholders::_1));

	// Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

	// Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / PLANNING_FREQUENCY)),
        std::bind(&PlannerNode::timer_callback, this));
}

void PlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    current_goal_ = msg;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

    planPath();
}

void PlannerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
}

void PlannerNode::timer_callback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
		if (goalReached()) {
			RCLCPP_INFO(this->get_logger(), "Goal reached!");
			state_ = State::WAITING_FOR_GOAL;
		} else {
			RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
			planPath();
		}
	}
}

bool PlannerNode::goalReached() const {
    if (!current_goal_) {
        return false;
    }
    double dx = current_goal_->point.x - current_pose_.position.x;
    double dy = current_goal_->point.y - current_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < GOAL_THRESHOLD; // return if difference is less than our tol
}

void PlannerNode::planPath() {
    if (!current_map_ || !current_goal_) {
		RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    auto result = planner_.planPath(current_map_, current_pose_, current_goal_->point);

    if (result) {
        path = *result;
        path_pub_->publish(path);
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
