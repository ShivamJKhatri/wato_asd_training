#include <algorithm>
#include <cmath>
#include <vector>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initGrid(double resolution, int width, int height,
                             double origin_x, double origin_y)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  // FREE rather than UNKNOWN: unexplored space reads as open, which the
  // assignment expects ("the planner may route through obstacles it hasn't
  // seen yet") and which keeps /map free of unexplored grey.
  const size_t cells = static_cast<size_t>(width_) * static_cast<size_t>(height_);
  grid_.assign(cells, FREE);
  scratch_.assign(cells, UNKNOWN);

  RCLCPP_INFO(logger_, "Global map: %dx%d cells at %.2f m/cell, origin (%.2f, %.2f)",
              width_, height_, resolution_, origin_x_, origin_y_);
}

bool MapMemoryCore::worldToGrid(double x, double y, int& grid_x, int& grid_y) const
{
  if (grid_.empty()) {
    return false;
  }

  // floor, not a plain cast: truncation folds points just outside the lower
  // edges back onto cell 0.
  const int gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  const int gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));

  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
    return false;
  }

  grid_x = gx;
  grid_y = gy;
  return true;
}

void MapMemoryCore::integrate(const nav_msgs::msg::OccupancyGrid& costmap,
                              double robot_x, double robot_y, double robot_yaw)
{
  if (grid_.empty() || costmap.data.empty()) {
    return;
  }

  const double cm_resolution = costmap.info.resolution;
  const int cm_width = static_cast<int>(costmap.info.width);
  const int cm_height = static_cast<int>(costmap.info.height);
  const double cm_origin_x = costmap.info.origin.position.x;
  const double cm_origin_y = costmap.info.origin.position.y;

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  std::fill(scratch_.begin(), scratch_.end(), UNKNOWN);

  for (int cy = 0; cy < cm_height; ++cy) {
    // Costmap cell centres, in the robot frame.
    const double local_y = cm_origin_y + (cy + 0.5) * cm_resolution;

    for (int cx = 0; cx < cm_width; ++cx) {
      const int8_t value = costmap.data[static_cast<size_t>(cy) * cm_width + cx];

      // The cell the scan could not see. Whatever the map already knows about
      // it is better than anything this costmap can say.
      if (value == UNKNOWN) {
        continue;
      }

      const double local_x = cm_origin_x + (cx + 0.5) * cm_resolution;

      // Rigid transform from the robot frame into the world. The costmap is
      // expressed in the same frame odometry reports, so the robot's pose is
      // the whole transform -- no TF lookup needed.
      const double world_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
      const double world_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

      int grid_x = 0;
      int grid_y = 0;
      if (!worldToGrid(world_x, world_y, grid_x, grid_y)) {
        continue;
      }

      // The global map is coarser than the costmap, so roughly four costmap
      // cells land in each map cell. Keep the most dangerous of them.
      int8_t & cell = scratch_[static_cast<size_t>(grid_y) * width_ + grid_x];
      if (value > cell) {
        cell = value;
      }
    }
  }

  // Commit by keeping the higher cost. The costmap reports free for anything
  // it is not currently hitting, including obstacles hidden behind others, so
  // letting it overwrite would erase what the map already learned. Every
  // obstacle here is static, so one can never legitimately vanish.
  for (size_t i = 0; i < grid_.size(); ++i) {
    if (scratch_[i] != UNKNOWN && scratch_[i] > grid_[i]) {
      grid_[i] = scratch_[i];
    }
  }
}

}
