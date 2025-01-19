#include "control_node.hpp"

ControlNode::ControlNode()
    : Node("control_node"), path_received_(false), odom_received_(false),
      lookahead_distance_(1.0), max_linear_speed_(0.5), max_angular_speed_(1.0) {
    
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Control Node initialized");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg) {
    current_path_ = *path_msg;
    path_received_ = true;
    computeControlCommands();
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    current_pose_ = odom_msg->pose.pose;
    odom_received_ = true;
    if (path_received_) {
        computeControlCommands();
    }
}

void ControlNode::computeControlCommands() {
    if (current_path_.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Path is empty, stopping the robot.");
        geometry_msgs::msg::Twist stop_msg;
        cmd_vel_pub_->publish(stop_msg);
        return;
    }

    // Find the closest waypoint within the lookahead distance
    geometry_msgs::msg::PoseStamped target_pose;
    bool target_found = false;

    for (const auto &pose : current_path_.poses) {
        double dx = pose.pose.position.x - current_pose_.position.x;
        double dy = pose.pose.position.y - current_pose_.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance > lookahead_distance_) {
            target_pose = pose;
            target_found = true;
            break;
        }
    }

    if (!target_found) {
        RCLCPP_WARN(this->get_logger(), "No target waypoint found within lookahead distance.");
        return;
    }

    // Compute control commands using Pure Pursuit
    double dx = target_pose.pose.position.x - current_pose_.position.x;
    double dy = target_pose.pose.position.y - current_pose_.position.y;

    double yaw = std::atan2(2.0 * (current_pose_.orientation.w * current_pose_.orientation.z),
                            1.0 - 2.0 * (current_pose_.orientation.z * current_pose_.orientation.z));

    double target_angle = std::atan2(dy, dx);
    double angle_diff = target_angle - yaw;

    // Normalize angle to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = std::min(max_linear_speed_, std::sqrt(dx * dx + dy * dy));
    cmd_vel.angular.z = std::min(max_angular_speed_, angle_diff);

    cmd_vel_pub_->publish(cmd_vel);
    RCLCPP_INFO(this->get_logger(), "Published velocity command: linear=%.2f, angular=%.2f",
                cmd_vel.linear.x, cmd_vel.angular.z);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
