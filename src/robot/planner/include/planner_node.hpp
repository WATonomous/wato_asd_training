#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "planner_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class PlannerNode : public rclcpp::Node {
public:
	PlannerNode();

private:
	enum class State {WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL};
	State state_{State::WAITING_FOR_GOAL};

	robot::PlannerCore planner_;

	// Subscribers and Publisher
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Data Storage
	nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
	geometry_msgs::msg::PointStamped::SharedPtr current_goal_;
	geometry_msgs::msg::Pose current_pose_;

	void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
	void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void timer_callback();

	bool goalReached() const;
	void planPath();

	static constexpr double GOAL_THRESHOLD = 0.5; // in meters
	static constexpr double PLANNING_FREQUENCY = 2.0;
};

#endif