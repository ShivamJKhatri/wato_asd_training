#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::configure(double lookahead, double max_linear, double max_angular,
                            double goal_tolerance, double turn_slowdown)
{
  lookahead_ = lookahead;
  max_linear_ = max_linear;
  max_angular_ = max_angular;
  goal_tolerance_ = goal_tolerance;
  turn_slowdown_ = turn_slowdown;

  RCLCPP_INFO(logger_, "Pure Pursuit: lookahead %.2f m, max %.2f m/s and %.2f rad/s",
              lookahead_, max_linear_, max_angular_);
}

std::pair<double, double> ControlCore::computeVelocity(
  const nav_msgs::msg::Path& path, double x, double y, double yaw) const
{
  // An empty path is how the planner says stop.
  if (path.poses.empty()) {
    return {0.0, 0.0};
  }

  const auto & end = path.poses.back().pose.position;
  if (std::hypot(end.x - x, end.y - y) < goal_tolerance_) {
    return {0.0, 0.0};
  }

  // Start from the waypoint nearest the robot. Scanning from the front of the
  // path instead would lock onto a point already behind us once the robot is
  // part-way along, and drive it backwards.
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    const double distance = std::hypot(p.x - x, p.y - y);
    if (distance < best) {
      best = distance;
      nearest = i;
    }
  }

  // The first waypoint at least a lookahead away, or the end of the path if
  // every remaining one is closer than that.
  size_t target = path.poses.size() - 1;
  for (size_t i = nearest; i < path.poses.size(); ++i) {
    const auto & p = path.poses[i].pose.position;
    if (std::hypot(p.x - x, p.y - y) >= lookahead_) {
      target = i;
      break;
    }
  }

  // Into the robot's own frame: +x straight ahead, +y to its left.
  const auto & goal = path.poses[target].pose.position;
  const double dx = goal.x - x;
  const double dy = goal.y - y;
  const double local_x = dx * std::cos(yaw) + dy * std::sin(yaw);
  const double local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);

  // Behind us. No amount of steering while moving forward reaches a point
  // behind the robot, so turn on the spot until it is in front.
  if (local_x <= 0.0) {
    return {0.0, std::copysign(max_angular_, local_y)};
  }

  // Pure Pursuit: the arc from here through the lookahead point has curvature
  // 2*local_y / L^2, and angular velocity is that curvature times speed.
  const double distance_squared = local_x * local_x + local_y * local_y;
  const double curvature = 2.0 * local_y / distance_squared;

  // Slow down in proportion to how hard the turn is. Holding full speed through
  // a tight corner is what makes the robot cut across it into the obstacle the
  // planner routed around; at a lower speed the same curvature is achievable.
  const double linear = max_linear_ / (1.0 + turn_slowdown_ * std::abs(curvature));
  const double angular = std::clamp(linear * curvature, -max_angular_, max_angular_);

  // ponytail: no ramp into the goal, it just stops at the tolerance. Add one if
  // it overshoots visibly.
  return {linear, angular};
}

}
