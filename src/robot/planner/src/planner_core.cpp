#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "planner_core.hpp"

namespace robot
{
namespace
{

// 8-connected, so a diagonal run is one straight line instead of a staircase.
constexpr int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

// A goal clicked near a wall lands in the inflation band. Rather than refuse,
// plan to the nearest cell the robot can actually occupy within this radius.
constexpr double kGoalSnapRadius = 3.0;

// A robot that has drifted into the inflation band is walled in by its own
// safety margin. Cells this close to it stay driveable so it can get out.
// Must exceed the costmap's inflation_radius.
constexpr double kEscapeRadius = 2.0;

// How hard inflation pushes the path away from walls. At the top of the
// gradient this adds ~4.9 to a step that otherwise costs 1, so the path hugs
// open space wherever open space exists.
constexpr double kCostWeight = 0.05;

bool toGrid(const nav_msgs::msg::OccupancyGrid& map, double x, double y, int& gx, int& gy)
{
  gx = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  gy = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  return gx >= 0 && gy >= 0 &&
         gx < static_cast<int>(map.info.width) &&
         gy < static_cast<int>(map.info.height);
}

}  // namespace

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

std::vector<std::pair<double, double>> PlannerCore::plan(
  const nav_msgs::msg::OccupancyGrid& map,
  double start_x, double start_y, double goal_x, double goal_y) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;

  int sx = 0, sy = 0, gx = 0, gy = 0;
  if (map.data.empty() || resolution <= 0.0 ||
      !toGrid(map, start_x, start_y, sx, sy) ||
      !toGrid(map, goal_x, goal_y, gx, gy)) {
    return {};
  }

  const int start = sy * width + sx;
  int goal = gy * width + gx;

  // Clicking near a wall puts the goal inside the inflation band. Snap to the
  // closest cell that is actually driveable instead of giving up, which is
  // what made the robot stop dead part-way to a goal near an obstacle.
  if (map.data[goal] >= BLOCKED) {
    const int reach = static_cast<int>(std::ceil(kGoalSnapRadius / resolution));
    int nearest = -1;
    double nearest_distance = std::numeric_limits<double>::infinity();

    for (int dy = -reach; dy <= reach; ++dy) {
      for (int dx = -reach; dx <= reach; ++dx) {
        const int nx = gx + dx;
        const int ny = gy + dy;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        const int index = ny * width + nx;
        if (map.data[index] < 0 || map.data[index] >= BLOCKED) {
          continue;
        }
        const double distance = std::hypot(dx, dy);
        if (distance < nearest_distance) {
          nearest_distance = distance;
          nearest = index;
        }
      }
    }

    if (nearest < 0) {
      return {};  // Nothing driveable anywhere near the goal.
    }
    goal = nearest;
    gx = goal % width;
    gy = goal / width;
  }

  // How far from the start the inflation band stays driveable.
  const int escape = static_cast<int>(std::ceil(kEscapeRadius / resolution));

  std::vector<double> cost_so_far(map.data.size(), std::numeric_limits<double>::infinity());
  std::vector<int> came_from(map.data.size(), -1);

  const auto heuristic = [&](int index) {
    return std::hypot(index % width - gx, index / width - gy);
  };

  // (f-score, cell). greater<> turns the max-heap into a min-heap.
  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>,
                      std::greater<>> open;

  cost_so_far[start] = 0.0;
  open.push({heuristic(start), start});

  while (!open.empty()) {
    const double f = open.top().first;
    const int current = open.top().second;
    open.pop();

    if (current == goal) {
      break;
    }
    // A cheaper route to this cell was found after this entry was queued.
    if (f > cost_so_far[current] + heuristic(current)) {
      continue;
    }

    const int cx = current % width;
    const int cy = current / width;

    for (int k = 0; k < 8; ++k) {
      const int nx = cx + kDx[k];
      const int ny = cy + kDy[k];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }

      const int next = ny * width + nx;
      const int8_t cell = map.data[next];
      if (cell < 0 || cell >= LETHAL) {
        continue;  // An obstacle is never driveable.
      }

      // Inflation is off limits, except right around the robot, so that one
      // which has ended up inside the band can plan its way back out.
      const bool near_start = std::abs(nx - sx) <= escape && std::abs(ny - sy) <= escape;
      if (cell >= BLOCKED && !near_start) {
        continue;
      }

      // Diagonals are longer, and inflation makes a cell more expensive to
      // cross without forbidding it outright.
      const double step = (kDx[k] != 0 && kDy[k] != 0 ? M_SQRT2 : 1.0) + cell * kCostWeight;
      const double tentative = cost_so_far[current] + step;

      if (tentative < cost_so_far[next]) {
        cost_so_far[next] = tentative;
        came_from[next] = current;
        open.push({tentative + heuristic(next), next});
      }
    }
  }

  if (goal != start && came_from[goal] < 0) {
    return {};  // Never reached.
  }

  std::vector<std::pair<double, double>> path;
  for (int i = goal; i >= 0; i = came_from[i]) {
    path.push_back({map.info.origin.position.x + (i % width + 0.5) * resolution,
                    map.info.origin.position.y + (i / width + 0.5) * resolution});
    if (i == start) {
      break;
    }
  }
  std::reverse(path.begin(), path.end());

  return path;
}

}
