#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
    processParameters();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        goal_topic_, 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

    planner_.initPlanner(smoothing_factor_, iterations_);
}

void PlannerNode::processParameters() {
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
    this->declare_parameter<std::string>("path_topic", "/path");
    this->declare_parameter<double>("smoothing_factor", 0.2);
    this->declare_parameter<int>("iterations", 20);
    this->declare_parameter<double>("goal_tolerance", 0.3);
    this->declare_parameter<double>("plan_timeout_seconds", 10.0);

    map_topic_ = this->get_parameter("map_topic").as_string();
    goal_topic_ = this->get_parameter("goal_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    iterations_ = this->get_parameter("iterations").as_int();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    plan_timeout_ = this->get_parameter("plan_timeout_seconds").as_double();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = msg;
    }

    if (active_goal_) {
        double elapsed = (now() - plan_start_time_).seconds();
        if (elapsed <= plan_timeout_) {
            RCLCPP_INFO(this->get_logger(), "Replanning for goal (elapsed: %.2f sec).", elapsed);
            publishPath();
        }
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal_msg) {
    if (active_goal_) {
        RCLCPP_WARN(this->get_logger(), "New goal received. Replacing current goal.");
        resetGoal();
    }

    current_goal_ = *goal_msg;
    active_goal_ = true;
    plan_start_time_ = now();

    RCLCPP_INFO(this->get_logger(), "Accepted new goal: (%.2f, %.2f)", goal_msg->point.x, goal_msg->point.y);
    publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    odom_x_ = odom_msg->pose.pose.position.x;
    odom_y_ = odom_msg->pose.pose.position.y;
    have_odom_ = true;
}

void PlannerNode::timerCallback() {
    if (!active_goal_) {
        return;
    }

    double elapsed = (now() - plan_start_time_).seconds();
    double distance_to_goal = sqrt(pow(odom_x_ - current_goal_.point.x, 2) + pow(odom_y_ - current_goal_.point.y, 2));

    if (elapsed > plan_timeout_) {
        RCLCPP_WARN(this->get_logger(), "Plan timed out. Resetting goal.");
        resetGoal();
        return;
    }

    if (distance_to_goal < goal_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached!");
        resetGoal();
        return;
    }

    double deviation_threshold = 1.0;
    double planned_distance = sqrt(pow(planned_x_ - odom_x_, 2) + pow(planned_y_ - odom_y_, 2));

    if (planned_distance > deviation_threshold) {
        RCLCPP_WARN(this->get_logger(), "Robot deviated. Replanning...");
        publishPath();
    }
}

void PlannerNode::publishPath() {
    if (!have_odom_) {
        RCLCPP_WARN(this->get_logger(), "No odometry received yet.");
        resetGoal();
        return;
    }

    double start_x = odom_x_;
    double start_y = odom_y_;

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!planner_.planPath(start_x, start_y, current_goal_.point.x, current_goal_.point.y, map_)) {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
            resetGoal();
            return;
        }
    }

    nav_msgs::msg::Path path_msg = *planner_.getPath();
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = map_->header.frame_id;
    path_pub_->publish(path_msg);

    if (!path_msg.poses.empty()) {
        planned_x_ = path_msg.poses[0].pose.position.x;
        planned_y_ = path_msg.poses[0].pose.position.y;
    }
}

void PlannerNode::resetGoal() {
    active_goal_ = false;
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    path_pub_->publish(empty_path);
    RCLCPP_INFO(this->get_logger(), "Published empty path to stop the robot.");
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
