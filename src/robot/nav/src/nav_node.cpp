#include <memory>

#include "nav_node.hpp"

NavNode::NavNode(): Node("navigation"), nav_(robot::NavCore())
{
  occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/occupancy", 20, 
    std::bind(&NavNode::occupancy_callback, this, std::placeholders::_1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/path_marker", 10
  );

  goal_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 20, 
    std::bind(&NavNode::goal_point_callback, this, std::placeholders::_1));

  next_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/next_point", 20);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
}

void NavNode::goal_point_callback(geometry_msgs::msg::PointStamped::SharedPtr point){
  this->goal_point = *point;
}

void NavNode::occupancy_callback(nav_msgs::msg::OccupancyGrid::SharedPtr occupancy){
  geometry_msgs::msg::TransformStamped robot_to_world;
  geometry_msgs::msg::TransformStamped world_to_robot;

  try {
    robot_to_world = tf_buffer_->lookupTransform("sim_world", "robot/chassis/gpu_lidar", tf2::TimePointZero);
    world_to_robot = tf_buffer_->lookupTransform("robot/chassis/gpu_lidar", "sim_world", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s", ex.what());
  }


  auto local_goal_point = geometry_msgs::msg::PointStamped();
  tf2::doTransform(this->goal_point, local_goal_point, world_to_robot);

  auto points = nav_.navigate(occupancy, local_goal_point.point);

  std::vector<visualization_msgs::msg::Marker> markers;

  int id = 0;
  for(auto point : points){
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "sim_world";

    marker.id = id;
    marker.type = 2; // Sphere

    geometry_msgs::msg::PointStamped point_stamped;
    point_stamped.header.frame_id = "robot/chassis/gpu_lidar";
    point_stamped.point = point;

    auto global_point = geometry_msgs::msg::PointStamped();

    tf2::doTransform(point_stamped, global_point, robot_to_world);

    if(id == points.size()-2){
      next_point_pub_->publish(global_point);
    }

    marker.pose.position = global_point.point;

    marker.scale.x = .5;
    marker.scale.y = .5;
    marker.scale.z = .5;

    markers.push_back(marker);

    id++;
  }

  auto marker_array = visualization_msgs::msg::MarkerArray();
  marker_array.markers = markers;

  marker_pub_->publish(marker_array);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavNode>());
  rclcpp::shutdown();
  return 0;
}
