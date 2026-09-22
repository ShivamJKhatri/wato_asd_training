#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    // Matches costmap's contract. UNKNOWN only ever arrives from a costmap --
    // the global map itself starts FREE and never holds an unknown cell, so
    // /map renders with no unexplored regions at any point.
    static constexpr int8_t UNKNOWN = -1;
    static constexpr int8_t FREE = 0;

    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Allocates the global map. Unlike the costmap this grid never moves: its
    // origin is fixed in the world frame.
    void initGrid(double resolution, int width, int height,
                  double origin_x, double origin_y);

    // Converts a point in the world frame (metres) into grid indices.
    // Returns false, leaving the outputs untouched, if the point is off-map.
    bool worldToGrid(double x, double y, int& grid_x, int& grid_y) const;

    // Merges one costmap into the global map, given the robot's pose in the
    // world frame. Cells the costmap reports as UNKNOWN keep whatever the map
    // already had, which is what stops an obstacle being erased once it passes
    // out of the sensor's view.
    void integrate(const nav_msgs::msg::OccupancyGrid& costmap,
                   double robot_x, double robot_y, double robot_yaw);

    double resolution() const { return resolution_; }
    int width() const { return width_; }
    int height() const { return height_; }
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }

    const std::vector<int8_t>& data() const { return grid_; }

  private:
    rclcpp::Logger logger_;

    double resolution_ = 0.0;
    int width_ = 0;
    int height_ = 0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    std::vector<int8_t> grid_;

    // Holds one fusion pass before it is committed, so that several costmap
    // cells landing in the same map cell resolve by cost rather than by
    // whichever happened to be written last.
    std::vector<int8_t> scratch_;
};

}

#endif
