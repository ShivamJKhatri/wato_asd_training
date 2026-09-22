#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Runs on a timer rather than in the callbacks, so fusion never blocks the
    // sensor stream and cannot run faster than the map needs.
    void updateMap();

    // Not const: stamping the header reads the node's clock.
    nav_msgs::msg::OccupancyGrid buildMessage();

    robot::MapMemoryCore map_memory_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool have_costmap_ = false;

    // Latest pose, and the pose at the last fusion.
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    bool have_odom_ = false;

    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double last_yaw_ = 0.0;
    bool have_fused_ = false;

    double update_distance_ = 1.5;
    double update_angle_ = 0.5;
    std::string global_frame_ = "sim_world";

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
