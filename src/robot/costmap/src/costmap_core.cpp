#include <algorithm>
#include <cmath>

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

void CostmapCore::reset()
{
  std::fill(grid_.begin(), grid_.end(), FREE);
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

  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
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

  grid_[static_cast<size_t>(grid_y) * static_cast<size_t>(width_) +
        static_cast<size_t>(grid_x)] = LETHAL;
}

}
