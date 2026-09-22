#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Cell values. This is a contract shared with map_memory and planner, so it
    // lives here rather than in the node.
    static constexpr int8_t FREE = 0;
    static constexpr int8_t LETHAL = 100;

    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Allocates the grid and centres it on the robot.
    void initGrid(double resolution, int width, int height);

    // Clears every cell back to FREE without reallocating.
    void reset();

    // Converts a point in the robot frame (metres) into grid indices.
    // Returns false, leaving the outputs untouched, if the point falls outside
    // the grid.
    bool worldToGrid(double x, double y, int& grid_x, int& grid_y) const;

    // Marks the cell containing (x, y) as an obstacle. Silently does nothing if
    // the point is outside the grid.
    void markObstacle(double x, double y);

    double resolution() const { return resolution_; }
    int width() const { return width_; }
    int height() const { return height_; }

    // The grid's bottom-left corner, in the robot frame, in metres.
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }

    // Row-major, matching nav_msgs/OccupancyGrid: index = y * width + x.
    const std::vector<int8_t>& data() const { return grid_; }

  private:
    rclcpp::Logger logger_;

    double resolution_ = 0.0;
    int width_ = 0;
    int height_ = 0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    std::vector<int8_t> grid_;
};

}

#endif
