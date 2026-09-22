#include <cmath>
#include <cstdint>
#include <memory>

#include "costmap_node.hpp"

namespace
{
// A 20 m x 20 m window at 0.1 m per cell, centred on the robot. The lidar
// reaches 20 m, but its beams are ~0.5 m apart out there, which draws walls as
// dotted lines; 10 m is where the returns are still dense enough to be solid.
// These become ROS parameters in stage 2.
constexpr double kResolution = 0.1;
constexpr int kGridWidth = 200;
constexpr int kGridHeight = 200;
}  // namespace

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  costmap_.initGrid(kResolution, kGridWidth, kGridHeight);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // The costmap holds only what this scan sees, so every scan starts clean.
  costmap_.reset();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const float range = scan->ranges[i];

    // Gazebo reports inf for beams that hit nothing, and readings outside the
    // sensor's own limits are not measurements.
    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }

    // Polar to Cartesian, in the lidar's frame: beam i points at angle_min plus
    // i steps of angle_increment.
    const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    costmap_.markObstacle(range * std::cos(angle), range * std::sin(angle));
  }

  costmap_pub_->publish(buildMessage(scan->header));
}

nav_msgs::msg::OccupancyGrid CostmapNode::buildMessage(
  const std_msgs::msg::Header& header) const {
  nav_msgs::msg::OccupancyGrid msg;

  // Reuse the scan's stamp and frame: the grid describes the world at the
  // moment of that scan, not at wall-clock now().
  msg.header = header;

  msg.info.resolution = static_cast<float>(costmap_.resolution());
  msg.info.width = static_cast<uint32_t>(costmap_.width());
  msg.info.height = static_cast<uint32_t>(costmap_.height());
  msg.info.origin.position.x = costmap_.originX();
  msg.info.origin.position.y = costmap_.originY();
  // An all-zero quaternion is invalid; the grid is axis-aligned with the frame.
  msg.info.origin.orientation.w = 1.0;

  msg.data = costmap_.data();

  return msg;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
