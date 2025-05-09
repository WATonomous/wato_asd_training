#include <cmath>
#include "control_node.hpp"


PurePursuitController::PurePursuitController()
  : Node("pure_pursuit_controller"),
    lookahead_distance_(1.0),  // Example default value
    goal_tolerance_(0.1),      // Example default value
    linear_speed_(0.5),        // Example default value
    angular_speed_(0.5) {      // Example default value

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller Node Initialized");

    // Subscribers and Publishers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PurePursuitController::control_loop, this));
}

void PurePursuitController::control_loop() {
    if (!current_path_ || !robot_odom_) {
        RCLCPP_WARN(this->get_logger(), "Path or odometry data not available.");
        return;
    }

    auto robot_position = robot_odom_->pose.pose.position;
    auto robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

    // RCLCPP_INFO(this->get_logger(), "Robot Position: x=%f, y=%f, yaw=%f", robot_position.x, robot_position.y, robot_yaw);

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        RCLCPP_INFO(this->get_logger(), "No valid lookahead point found.");
        return;
    }

    // RCLCPP_INFO(this->get_logger(), "Lookahead Point: x=%f, y=%f",
    //              lookahead_point->pose.position.x, lookahead_point->pose.position.y);

    auto cmd_vel = computeVelocity(*lookahead_point);

    // RCLCPP_INFO(this->get_logger(), "Computed Twist: linear.x=%f, angular.z=%f",
    //              cmd_vel.linear.x, cmd_vel.angular.z);

    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> PurePursuitController::findLookaheadPoint() {
    if (!current_path_ || current_path_->poses.empty()) {
        return std::nullopt; // Return no lookahead point if the path is empty
    }

    auto robot_position = robot_odom_->pose.pose.position;
    auto robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

    double min_distance = std::numeric_limits<double>::max();
    int lookahead_index = 0;
    bool found_forward = false;

    // Loop through all path points to find the closest lookahead point
    for (size_t i = 0; i < current_path_->poses.size(); ++i) {
        double dx = current_path_->poses[i].pose.position.x - robot_position.x;
        double dy = current_path_->poses[i].pose.position.y - robot_position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Skip points that are too close (within lookahead distance)
        if (distance < lookahead_distance_) {
            continue;
        }

        // Calculate the angle to the lookahead point
        double angle_to_point = std::atan2(dy, dx);

        // Calculate the difference between the robot's heading and the angle to the lookahead point
        double angle_diff = angle_to_point - robot_yaw;

        // Normalize the angle difference to be within [-pi, pi]
        if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Check if the angle to the point is within the forward-facing direction range (-π/2 to π/2)
        if (std::abs(angle_diff) < M_PI / 2) {
            // We found a valid forward point, update the closest point
            if (distance < min_distance) {
                min_distance = distance;
                lookahead_index = i;
                found_forward = true;
            }
        }
    }

    // If no forward point was found, allow the robot to reverse
    if (!found_forward) {
        // Find the closest point regardless of direction
        for (size_t i = 0; i < current_path_->poses.size(); ++i) {
            double dx = current_path_->poses[i].pose.position.x - robot_position.x;
            double dy = current_path_->poses[i].pose.position.y - robot_position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            // Skip points that are too close
            if (distance < lookahead_distance_) {
                continue;
            }

            // Always select the closest point if no valid forward point was found
            if (distance < min_distance) {
                min_distance = distance;
                lookahead_index = i;
            }
        }
    }

    // Return the lookahead point if found
    if (lookahead_index < current_path_->poses.size()) {
        return current_path_->poses[lookahead_index];
    }

    return std::nullopt; // Return no lookahead point if none was found
}

geometry_msgs::msg::Twist PurePursuitController::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::Twist twist;

    // Get the robot's current position and orientation
    auto robot_position = robot_odom_->pose.pose.position;
    auto robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

    // Get the target (lookahead) point coordinates
    double lookahead_x = target.pose.position.x;
    double lookahead_y = target.pose.position.y;

    // Calculate the distance to the lookahead point
    double dx = lookahead_x - robot_position.x;
    double dy = lookahead_y - robot_position.y;

    // Calculate the angle to the lookahead point
    double angle_to_lookahead = std::atan2(dy, dx);

    // Calculate the steering angle (difference between robot's heading and lookahead direction)
    double steering_angle = angle_to_lookahead - robot_yaw;

    // Normalize the steering angle to the range [-pi, pi]
    if (steering_angle > M_PI) {
        steering_angle -= 2 * M_PI;
    } else if (steering_angle < -M_PI) {
        steering_angle += 2 * M_PI;
    }

    // If the steering angle is greater than the limit, stop linear motion
    if (std::abs(steering_angle) > angular_speed_) {
        twist.linear.x = 0.0;
    } else {
        twist.linear.x = linear_speed_;
    }

    // Limit the steering angle
    steering_angle = std::clamp(steering_angle, -angular_speed_, angular_speed_);

    // Set angular velocity based on the steering angle and a gain
    double angular_velocity = steering_angle * 1.0; // Gain for steering
    twist.angular.z = angular_velocity;

    return twist;
}

double PurePursuitController::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double PurePursuitController::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    // Only yaw is needed for the robot

    /* 
    My research on how quaternions work:

    Axis-Angle Representation of the Quaternion
    Axis of rotation: u = (ux, uy, uz))
    Angle of rotation: θ

    Quaternion representation of the rotation:
    q = (cos(θ/2), ux*sin(θ/2), uy*sin(θ/2), uz*sin(θ/2))

    BUT: yaw = rotation in z, so the unit vector u = (0, 0, 1)

    So, the quaternion representation of the yaw rotation is:

    q = (cos(θ/2), 0, 0, sin(θ/2))

    The angle θ can be extracted from the quaternion now 
    */

    // return 2 * std::acos(quat.w); // invalid becase it returns the absolute value of the angle (range of [0, pi])

    return 2 * std::atan2(quat.z, quat.w); // returns the angle in the range of [-pi, pi]
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}
