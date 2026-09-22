#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Defaults for a 20 m x 20 m window at 0.1 m per cell, centred on the robot.
  // params.yaml overrides these at launch; robot_radius and inflation_radius
  // can also be changed while running.
  this->declare_parameter("resolution", 0.1);
  this->declare_parameter("width", 200);
  this->declare_parameter("height", 200);
  this->declare_parameter("robot_radius", 0.7);
  this->declare_parameter("inflation_radius", 1.5);
  costmap_.initGrid(
    this->get_parameter("resolution").as_double(),
    static_cast<int>(this->get_parameter("width").as_int()),
    static_cast<int>(this->get_parameter("height").as_int()));

  costmap_.setInflation(
    this->get_parameter("robot_radius").as_double(),
    this->get_parameter("inflation_radius").as_double());

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  param_cb_ = this->add_on_set_parameters_callback(
    std::bind(&CostmapNode::parametersCallback, this, std::placeholders::_1));
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Each scan stands on its own: everything starts free, and only what this
  // scan actually hits becomes an obstacle.
  costmap_.reset();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const float range = scan->ranges[i];

    // Non-finite covers both inf (the beam hit nothing) and NaN (a broken
    // reading); neither marks an obstacle.
    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }

    // Polar to Cartesian, in the lidar's frame: beam i points at angle_min plus
    // i steps of angle_increment. Returns past the window edge are dropped by
    // the grid's own bounds check.
    const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    costmap_.markObstacle(range * std::cos(angle), range * std::sin(angle));
  }

  costmap_.inflate();

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

rcl_interfaces::msg::SetParametersResult CostmapNode::parametersCallback(
  const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  double robot_radius = this->get_parameter("robot_radius").as_double();
  double inflation_radius = this->get_parameter("inflation_radius").as_double();
  bool inflation_changed = false;

  for (const auto & parameter : parameters) {
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      continue;  // The grid dimensions are fixed once the node is running.
    }

    if (parameter.get_name() == "robot_radius") {
      robot_radius = parameter.as_double();
      inflation_changed = true;
    } else if (parameter.get_name() == "inflation_radius") {
      inflation_radius = parameter.as_double();
      inflation_changed = true;
    }
  }

  if (inflation_changed) {
    costmap_.setInflation(robot_radius, inflation_radius);
  }

  return result;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
