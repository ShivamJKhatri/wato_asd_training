#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Replans from scratch every tick while a goal is active. That covers both
    // "the map changed" and "the plan went stale" without tracking either.
    void replan();

    void publishPath(const std::vector<std::pair<double, double>>& waypoints);

    robot::PlannerCore planner_;

    nav_msgs::msg::OccupancyGrid map_;
    bool have_map_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool have_odom_ = false;

    double goal_x_ = 0.0;
    double goal_y_ = 0.0;

    // The whole state machine: waiting for a goal, or driving to one.
    bool goal_active_ = false;

    double goal_tolerance_ = 0.5;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
