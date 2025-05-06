#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode(double resolution, int grid_width, int grid_height,
                         int inflation_radius, int max_cost) 
                         : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())), 
                           resolution_(resolution), width_(grid_width),
                           height_(grid_height), inflation_radius_(inflation_radius),
                           max_cost_(max_cost) {
                            
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
}

void CostmapNode::initCostmap() {
  // RCLCPP_INFO(this->get_logger(), "Initializing costmap");

  costmap_grid_.clear(); // clear the previous costmap grid
  costmap_grid_.resize(height_, std::vector<int>(width_, 0)); // all cells init with 0

  // RCLCPP_INFO(this->get_logger(), "Costmap initialized with resolution: %f, width: %d, height: %d",
              // resolution_, width_, height_);
}

void CostmapNode::convert2Grid(double range, double angle, int& x_grid, int& y_grid) {
  // given by assignment docs -> real world coords in meters
  double x = range * std::cos(angle); 
  double y = range * std::sin(angle);

  x_grid = static_cast<int>((x - costmap_msg_.info.origin.position.x) / resolution_); 
  y_grid = static_cast<int>((y - costmap_msg_.info.origin.position.y) / resolution_);
}

void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
    // RCLCPP_INFO(this->get_logger(), "Marking obstacle at grid (%d, %d)", x_grid, y_grid);
  } else {
    // RCLCPP_WARN(this->get_logger(), "Grid coordinates (%d, %d) out of bounds", x_grid, y_grid);
    return;
  }
  costmap_grid_[y_grid][x_grid] = 100; // Mark as occupied "high cost (e.g., 100 for "occupied")."
}

double CostmapNode::dist2Point(int x1, int y1, int x2, int y2) {
  return std::sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1))) * resolution_; // convert to meters
}

double CostmapNode::getCost(int x, int y, int new_x, int new_y) {
  double euclidean_dist = dist2Point(x, y, new_x, new_y);

  if (euclidean_dist > inflation_radius_) {
    return 0; // outside the inflation radius
  }
  return (max_cost_ * (1.0 - (euclidean_dist / inflation_radius_))); // defined in assignment docs
}

void CostmapNode::inflateObstacles() {
  // RCLCPP_INFO(this->get_logger(), "Inflating obstacles");

  int inflation_radius_cells = static_cast<int>(inflation_radius_ / resolution_); // convert in units of # cells

  // iterate through the entire costmap
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {

      // if the cell is marked with max cost
      if (costmap_grid_[y][x] == max_cost_) {

        // inflate the surrounding cells (within the inflation radius of the current cell)
        for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy) {
          for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx) {

            int new_x = x + dx;
            int new_y = y + dy;

            // Check if the new coordinates are within the bounds of the costmap and inflation radius
            if (new_x >= 0 && new_x < width_ && new_y >= 0 && new_y < height_) { 
              
              double euclidean_dist = dist2Point(x, y, new_x, new_y) * resolution_; // convert to meters

              if (euclidean_dist <= inflation_radius_) {
                int calculated_cost = getCost(x, y, new_x, new_y); 
                // RCLCPP_INFO(this->get_logger(), "(%d, %d), (%d, %d), cost: %d dist %f", x, y, new_x, new_y, calculated_cost, euclidean_dist);

                if (calculated_cost > costmap_grid_[new_y][new_x]) {
                  costmap_grid_[new_y][new_x] = calculated_cost; 
                }
              }
            }
          }
        }
      } 
    }
  }
}

void CostmapNode::publishCostmap() {

  costmap_msg_.header.frame_id = "robot/chassis/lidar"; // frame_id for the costmap
  costmap_msg_.header.stamp = this->now();

  costmap_msg_.info.resolution = resolution_; 
  costmap_msg_.info.width = width_;
  costmap_msg_.info.height = height_;
  costmap_msg_.info.origin.position.x = -(0.5 * width_ * resolution_); 
  costmap_msg_.info.origin.position.y = -(0.5 * height_ * resolution_); 

  costmap_msg_.data.clear(); // empty the data vector before filling it
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
        costmap_msg_.data.push_back(costmap_grid_[y][x]);
    }
  }

  costmap_pub_->publish(costmap_msg_); // Publish the costmap message
  RCLCPP_INFO(this->get_logger(), "Costmap published");
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // RCLCPP_INFO(this->get_logger(), "Lidar scan data received");

  initCostmap(); 

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
        // Calculate grid coordinates
        int x_grid, y_grid;
        convert2Grid(range, angle, x_grid, y_grid);
        markObstacle(x_grid, y_grid);
    }
  }

  // RCLCPP_INFO(this->get_logger(), "Lidar scan data processed");

  inflateObstacles(); // Inflate the obstacles after marking them
  publishCostmap(); // Publish the costmap after inflating
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  double resolution = 0.1; // meters
  int grid_width = 300; // # of cells in x direction
  int grid_height = 300; // # of cells in y direction
  int inflation_radius = 1; // in meters
  int max_cost = 100; // max cost for occupied cells

  auto costmap_node = std::make_shared<CostmapNode>(resolution, grid_width, grid_height, inflation_radius, max_cost);

  rclcpp::spin(costmap_node);
  rclcpp::shutdown();
  return 0;
}