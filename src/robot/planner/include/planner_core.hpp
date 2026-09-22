#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class PlannerCore {
  public:
    // At or above this cost the robot's body would be touching an obstacle.
    // Matches CostmapCore::INSCRIBED.
    static constexpr int8_t BLOCKED = 99;

    // An actual laser return. Never driveable, under any circumstance.
    static constexpr int8_t LETHAL = 100;

    explicit PlannerCore(const rclcpp::Logger& logger);

    // A* from start to goal, both in the map's frame. Returns the waypoints in
    // world coordinates, start first and goal last, or empty if there is no
    // route.
    std::vector<std::pair<double, double>> plan(
      const nav_msgs::msg::OccupancyGrid& map,
      double start_x, double start_y, double goal_x, double goal_y) const;

  private:
    rclcpp::Logger logger_;
};

}

#endif
