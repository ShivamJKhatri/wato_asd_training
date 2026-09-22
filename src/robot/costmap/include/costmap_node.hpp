#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    // Rebuilds the costmap from a single scan and publishes it.
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Wraps the core's grid in an OccupancyGrid, reusing the scan's header.
    nav_msgs::msg::OccupancyGrid buildMessage(const std_msgs::msg::Header& header) const;

    // Lets the inflation radii be retuned with `ros2 param set` instead of a
    // rebuild, which is slow here.
    rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

    robot::CostmapCore costmap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

#endif
