#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"

using std::placeholders::_1;
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::publishCostmap, this, _1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&CostmapNode::setPose, this, _1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  past_ = this->get_clock()->now();
}

void CostmapNode::publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  /*
    This formula below ensures that at the maximum distance,
    the maximum gap between laser scans is exactly one cell
  */
  int res = 2 / scan->angle_increment;

  /* Scan */
  std::vector<int8_t> grid(res * res, 0);
  for (int i = 0; i < scan->ranges.size(); i++) {
    double angle = scan->angle_min + i * scan->angle_increment + robot_angle;
    double range = scan->ranges[i];
    if (range > scan->range_min && range < scan->range_max) {
      int grid_x = res / 2 + (range / scan->range_max) * -sin(angle) * (res / 2);
      int grid_y = res / 2 + (range / scan->range_max) * -cos(angle) * (res / 2);

      grid[res * grid_y + grid_x] = 100;
    }
  }

  inflate(grid, res, 2.1);

  /* Set up OccupancyGrid */
  auto occgrid = nav_msgs::msg::OccupancyGrid();
  occgrid.data = grid;
  occgrid.info.map_load_time = this->get_clock()->now();
  occgrid.info.resolution = 2 * scan->range_max / res;
  occgrid.info.width = res;
  occgrid.info.height = res;
  occgrid.info.origin.position.x = robot_x - scan->range_max;
  occgrid.info.origin.position.y = robot_y - scan->range_max;
  occgrid.info.origin.position.z = robot_z;
  occgrid.info.origin.orientation.x = 0;
  occgrid.info.origin.orientation.y = 0;
  occgrid.info.origin.orientation.z = 0;
  occgrid.info.origin.orientation.w = 1.0;
  
  costmap_pub_->publish(occgrid);
  
  /* Print grid */
  // auto clock = this->get_clock()->now();
  // if (clock > past_ + rclcpp::Duration::from_seconds(2))
  // {
  //   past = clock;
  //   std::string out = "";
  //   for (int i = 0; i < res; i++) {
  //     for (int j = 0; j < res; j++) {
  //       if (i == res / 2 && j == res / 2)
  //         out += "@@";
  //       else if (grid[res * i + j] == 100)
  //         out += "██";
  //       else if (grid[res * i + j] > 0)
  //         out += "##";
  //       else
  //         out += "  ";
  //     }
  //     out += "\n";
  //   }
  //   print(out);
  // }
}

void CostmapNode::inflate(std::vector<int8_t>& grid, int res, double radius) {
  int intrad = ceil(radius);
  for (int y = 0; y < res; y++) {
    for (int x = 0; x < res; x++) {
      if (grid[y * res + x] == 100) {
        for (int i = -intrad; i <= intrad; i++) {
          for (int j = -intrad; j <= intrad; j++) {
            double dist = sqrt(i * i + j * j);

            if (dist < radius) {
              int curcost = get(grid, res, y + i, x + j);
              int newcost = 100.0 * (1.0 - dist / radius);

              if (newcost > curcost) {
                set(grid, res, y + i, x + j, newcost);
              }
            }
          }
        }
      }
    }
  }
}
void CostmapNode::set(std::vector<int8_t>& grid, int res, int y, int x, int8_t val) {
  if (y >= 0 && y < res && x >= 0 && x < res) {
    grid[res * y + x] = val;
  }
}
int8_t CostmapNode::get(std::vector<int8_t> grid, int res, int y, int x) {
  if (y >= 0 && y < res && x >= 0 && x < res)
    return grid[res * y + x];
  return 100;
}

void CostmapNode::setPose(const nav_msgs::msg::Odometry::SharedPtr odom) {
  tf2::Quaternion quat_tf;
  tf2::fromMsg(odom->pose.pose.orientation, quat_tf);
  double r{}, p{}, y{};
  tf2::Matrix3x3 m(quat_tf);
  m.getRPY(r, p, y);
  robot_angle = y;
  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  robot_z = odom->pose.pose.position.z;
}

void CostmapNode::print(std::string s) {
  auto message = std_msgs::msg::String();
  message.data = s;
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}