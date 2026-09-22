#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::initGrid(double resolution, int width, int height)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;

  // The origin is the grid's bottom-left corner. Putting it half a grid away in
  // each negative direction leaves the robot sitting at the centre.
  origin_x_ = -(width_ * resolution_) / 2.0;
  origin_y_ = -(height_ * resolution_) / 2.0;

  grid_.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), FREE);

  RCLCPP_INFO(logger_, "Costmap: %dx%d cells at %.2f m/cell, origin (%.2f, %.2f)",
              width_, height_, resolution_, origin_x_, origin_y_);
}

void CostmapCore::setInflation(double robot_radius, double inflation_radius)
{
  inflation_kernel_.clear();

  if (resolution_ <= 0.0 || inflation_radius <= 0.0) {
    RCLCPP_WARN(logger_, "Inflation disabled: radius %.2f m at %.2f m/cell",
                inflation_radius, resolution_);
    return;
  }

  const int reach = static_cast<int>(std::ceil(inflation_radius / resolution_));

  for (int dy = -reach; dy <= reach; ++dy) {
    for (int dx = -reach; dx <= reach; ++dx) {
      // The obstacle cell itself keeps LETHAL; the disc only covers its
      // surroundings.
      if (dx == 0 && dy == 0) {
        continue;
      }

      const double distance = std::hypot(dx, dy) * resolution_;
      if (distance > inflation_radius) {
        continue;  // The disc is round, the loop above is square.
      }

      int8_t cost = INSCRIBED;
      if (distance > robot_radius && inflation_radius > robot_radius) {
        // Fade from INSCRIBED at the robot's own radius down to nothing at the
        // edge of the disc.
        const double t = (distance - robot_radius) / (inflation_radius - robot_radius);
        const long faded = std::lround(INSCRIBED * (1.0 - t));
        if (faded < 1) {
          continue;  // Too cheap to be worth storing or stamping.
        }
        cost = static_cast<int8_t>(faded);
      }

      inflation_kernel_.push_back({dx, dy, cost});
    }
  }

  RCLCPP_INFO(logger_, "Inflation: %zu cells per obstacle (robot %.2f m, radius %.2f m)",
              inflation_kernel_.size(), robot_radius, inflation_radius);
}

void CostmapCore::reset()
{
  std::fill(grid_.begin(), grid_.end(), FREE);
}

bool CostmapCore::inBounds(int grid_x, int grid_y) const
{
  return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

int8_t& CostmapCore::at(int grid_x, int grid_y)
{
  return grid_[static_cast<size_t>(grid_y) * static_cast<size_t>(width_) +
               static_cast<size_t>(grid_x)];
}

bool CostmapCore::worldToGrid(double x, double y, int& grid_x, int& grid_y) const
{
  if (grid_.empty()) {
    return false;
  }

  // Measure the point from the grid's corner rather than from the robot, then
  // scale metres into cells.
  const double cell_x = (x - origin_x_) / resolution_;
  const double cell_y = (y - origin_y_) / resolution_;

  // floor, not a plain cast. Casting truncates toward zero, so a point just
  // outside the bottom-left corner (cell_x of -0.5) would become cell 0 and be
  // accepted as if it were inside the grid.
  const int gx = static_cast<int>(std::floor(cell_x));
  const int gy = static_cast<int>(std::floor(cell_y));

  if (!inBounds(gx, gy)) {
    return false;
  }

  grid_x = gx;
  grid_y = gy;
  return true;
}

void CostmapCore::markObstacle(double x, double y)
{
  int grid_x = 0;
  int grid_y = 0;
  if (!worldToGrid(x, y, grid_x, grid_y)) {
    return;  // Outside the costmap window, so there is no cell to mark.
  }

  at(grid_x, grid_y) = LETHAL;
}

void CostmapCore::inflate()
{
  if (inflation_kernel_.empty()) {
    return;
  }

  // Collect the obstacles before stamping anything. Inflating in place would
  // let a freshly written cost be mistaken for an obstacle and seed a second
  // round of inflation, smearing cost across the whole grid.
  std::vector<std::pair<int, int>> obstacles;
  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      if (at(gx, gy) == LETHAL) {
        obstacles.emplace_back(gx, gy);
      }
    }
  }

  for (const auto & obstacle : obstacles) {
    for (const auto & offset : inflation_kernel_) {
      const int gx = obstacle.first + offset.dx;
      const int gy = obstacle.second + offset.dy;
      if (!inBounds(gx, gy)) {
        continue;
      }

      // Never lower a cost: where two discs overlap, the higher one wins, so a
      // cell between two obstacles is as dangerous as the nearer one makes it.
      int8_t & cell = at(gx, gy);
      if (cell < offset.cost) {
        cell = offset.cost;
      }
    }
  }
}

}
