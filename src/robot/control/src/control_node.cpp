#include <memory>
#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore()) {

    goal_point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 
        20,
        std::bind(&ControlNode::goal_point_subscription_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControlNode::timer_callback, this)
    );
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
 

    



    // publisher_ = this->create_publisher<std_msgs::msg::String>("/example_string", 20);

    // subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "/model/robot/odometry", 
    //     20, 
    //     std::bind(&ControlNode::subscription_callback, this, std::placeholders::_1)
    // );


    // publisher_timer_ = this->create_wall_timer(
    //     std::chrono::milliseconds(1000), 
    //     std::bind(&ControlNode::publisher_timer_callback, this)
    // );
}

void ControlNode::timer_callback(){
    //RCLCPP_INFO(this->get_logger(), "x control: %f, y control: %f", Kp_linear*x, Kp_angular*y);

    if (!goal_point_) {
        RCLCPP_WARN(this->get_logger(), "Goal point not yet received");
        return; // Exit early if goal_point_ is not set
    }

    geometry_msgs::msg::Twist twist_msg;

    try {
        transform = tf_buffer_->lookupTransform("robot", "sim_world", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
    }

    // Create a transformed_point variable
    geometry_msgs::msg::PointStamped transformed_point;
    // Apply the transformation
    tf2::doTransform(*goal_point_, transformed_point, transform);

    RCLCPP_INFO(this->get_logger(), 
            "Transformed Goal Point's: X Position: %f, Y Position: %f, Z Position: %f", 
            transformed_point.point.x, 
            transformed_point.point.y, 
            transformed_point.point.z);

    // Set the linear x component
    twist_msg.linear.x = Kp_linear*transformed_point.point.x;

    // Set the angular z component
    twist_msg.angular.z = Kp_angular*transformed_point.point.y;

    // You can then publish this message using your publisher
    velocity_publisher_->publish(twist_msg);

}

void ControlNode::goal_point_subscription_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), 
                "Goal Point's: X Position: %f, Y Position: %f, Z Position: %f", 
                msg->point.x, 
                msg->point.y, 
                msg->point.z);

    goal_point_ = msg;
}


// void ControlNode::publisher_timer_callback() {
//     RCLCPP_INFO(this->get_logger(), "Timer is functional");
//     auto message = std_msgs::msg::String();
//     message.data = "Publisher is functional";
//     publisher_->publish(message);
// }

// void ControlNode::subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//     RCLCPP_INFO(this->get_logger(), 
//                 "Robot's X Position: %f, Robot's Y Position: %f, Robot's Z Position: %f", 
//                 msg->pose.pose.position.x, 
//                 msg->pose.pose.position.y, 
//                 msg->pose.pose.position.z);
//     //RCLCPP_INFO(this->get_logger(), "Message received");
// }



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
