#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  this->declare_parameter("goal_tolerance", 0.5);
  this->declare_parameter("plan_period", 0.5);

  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(this->get_parameter("plan_period").as_double()),
    std::bind(&PlannerNode::replan, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = *msg;
  have_map_ = true;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // The goal arrives from a click in Foxglove, stamped with whatever frame that
  // panel is displaying. If that is not the map's frame the coordinates mean
  // something else entirely, and the robot would drive somewhere surprising.
  if (have_map_ && !msg->header.frame_id.empty() &&
      msg->header.frame_id != map_.header.frame_id)
  {
    RCLCPP_WARN(this->get_logger(), "Goal is in frame '%s' but the map is in '%s'; ignoring",
                msg->header.frame_id.c_str(), map_.header.frame_id.c_str());
    return;
  }

  goal_x_ = msg->point.x;
  goal_y_ = msg->point.y;
  goal_active_ = true;

  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_x_, goal_y_);
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::replan()
{
  if (!goal_active_ || !have_map_ || !have_odom_) {
    return;
  }

  const auto waypoints = planner_.plan(map_, robot_x_, robot_y_, goal_x_, goal_y_);

  // Arrived if we are near the goal, or near the end of the plan -- a goal
  // clicked inside the inflation band gets snapped to the closest driveable
  // cell, and that snapped point is as close as the robot can legally get.
  const bool at_goal =
    std::hypot(goal_x_ - robot_x_, goal_y_ - robot_y_) < goal_tolerance_ ||
    (!waypoints.empty() &&
     std::hypot(waypoints.back().first - robot_x_,
                waypoints.back().second - robot_y_) < goal_tolerance_);

  if (at_goal) {
    goal_active_ = false;
    RCLCPP_INFO(this->get_logger(), "Goal reached");
    publishPath({});  // An empty path is how control is told to stop.
    return;
  }

  if (waypoints.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "No route to (%.2f, %.2f)", goal_x_, goal_y_);
  }

  publishPath(waypoints);
}

void PlannerNode::publishPath(const std::vector<std::pair<double, double>>& waypoints)
{
  nav_msgs::msg::Path msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = map_.header.frame_id;

  msg.poses.reserve(waypoints.size());
  for (const auto & point : waypoints) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }

  path_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
