#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Cell values. This is a contract shared with map_memory and planner.
    //
    //   UNKNOWN    never published by this node. It exists only so the value
    //              is named; nothing here ever emits it, which is what keeps
    //              grey out of every layer that renders a costmap.
    //   FREE       the default: anything no beam stopped in.
    //   1..98      inflation, rising with proximity to an obstacle.
    //   INSCRIBED  close enough that the robot's body would be touching.
    //   LETHAL     a beam stopped here.
    static constexpr int8_t UNKNOWN = -1;
    static constexpr int8_t FREE = 0;
    static constexpr int8_t INSCRIBED = 99;
    static constexpr int8_t LETHAL = 100;

    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Allocates the grid and centres it on the robot.
    void initGrid(double resolution, int width, int height);

    // Builds the disc of offsets used by inflate(). Cheap enough to call again
    // at runtime when the radii are retuned.
    void setInflation(double robot_radius, double inflation_radius);

    // Clears every cell back to FREE without reallocating.
    void reset();

    // Converts a point in the robot frame (metres) into grid indices.
    // Returns false, leaving the outputs untouched, if the point falls outside
    // the grid.
    bool worldToGrid(double x, double y, int& grid_x, int& grid_y) const;

    // Marks the cell containing (x, y) LETHAL. Silently does nothing if the
    // point is outside the grid.
    void markObstacle(double x, double y);

    // Spreads cost outward from every LETHAL cell. Call once per scan, after
    // the free space and obstacles are in. Never lowers a cell's cost, and
    // never writes into an UNKNOWN cell.
    void inflate();

    double resolution() const { return resolution_; }
    int width() const { return width_; }
    int height() const { return height_; }

    // The grid's bottom-left corner, in the robot frame, in metres.
    double originX() const { return origin_x_; }
    double originY() const { return origin_y_; }

    // Row-major, matching nav_msgs/OccupancyGrid: index = y * width + x.
    const std::vector<int8_t>& data() const { return grid_; }

  private:
    // One cell of the inflation disc: where it sits relative to an obstacle,
    // and what it costs to be there.
    struct InflationOffset {
      int dx;
      int dy;
      int8_t cost;
    };

    int8_t& at(int grid_x, int grid_y);
    bool inBounds(int grid_x, int grid_y) const;

    rclcpp::Logger logger_;

    double resolution_ = 0.0;
    int width_ = 0;
    int height_ = 0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    std::vector<int8_t> grid_;
    std::vector<InflationOffset> inflation_kernel_;
};

}

#endif
