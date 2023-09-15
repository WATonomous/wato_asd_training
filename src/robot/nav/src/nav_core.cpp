#include <string>
#include <vector>

#include "nav_core.hpp"

namespace robot
{
  NavCore::NavCore(){}

  std::vector<geometry_msgs::msg::Point> NavCore::navigate(nav_msgs::msg::OccupancyGrid::SharedPtr occupancy, geometry_msgs::msg::Point local_goal_point){
    int width = occupancy->info.width;
    int height = occupancy->info.height;
    float resolution = occupancy->info.resolution;
    float origin_x = occupancy->info.origin.position.x;
    float origin_y = occupancy->info.origin.position.y;

    int goal_x = std::round((local_goal_point.x / resolution) + (width / 2));
    int goal_y = std::round((local_goal_point.y / resolution) + (height / 2));

    AStar::Generator generator;

    generator.setWorldSize({width, height});

    for (int y = 0; y < height; y++){
      for (int x = 0; x < width; x++){
        auto data = occupancy->data[y*width+x];
        // Occupied
        if(data > 0){
          auto vec = AStar::Vec2i();
          vec.x = x;
          vec.y = y;
          generator.addCollision(vec);
        }
      }
    }

    // You can use a few heuristics : manhattan, euclidean or octagonal.
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    // This method returns vector of coordinates from target to source.
    auto path = generator.findPath({width/2, height/2}, {goal_x, goal_y});

    std::vector<geometry_msgs::msg::Point> points;

    for(auto& coordinate : path) {
      auto point = geometry_msgs::msg::Point();
      point.x = coordinate.x * resolution + origin_x;
      point.y = coordinate.y * resolution + origin_y;
      points.push_back(point);
    }

    return points;
  }
}  


