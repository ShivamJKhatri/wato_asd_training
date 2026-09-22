#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "control_core.hpp"

namespace
{

constexpr double kLookahead = 1.5;
constexpr double kMaxLinear = 1.0;
constexpr double kMaxAngular = 1.5;
constexpr double kTurnSlowdown = 2.0;
constexpr double kGoalTolerance = 0.5;

robot::ControlCore makeControl()
{
  robot::ControlCore control(rclcpp::get_logger("control_test"));
  control.configure(kLookahead, kMaxLinear, kMaxAngular, kGoalTolerance, kTurnSlowdown);
  return control;
}

nav_msgs::msg::Path makePath(const std::vector<std::pair<double, double>> & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "sim_world";
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

// A straight run along +x at 1 m spacing.
nav_msgs::msg::Path straightPath(int points = 11)
{
  std::vector<std::pair<double, double>> pts;
  for (int i = 0; i < points; ++i) {
    pts.push_back({static_cast<double>(i), 0.0});
  }
  return makePath(pts);
}

}  // namespace

// An empty path is the planner's stop signal, and it must actually stop.
TEST(ControlCoreTest, EmptyPathStops)
{
  const auto velocity = makeControl().computeVelocity(nav_msgs::msg::Path(), 0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(velocity.first, 0.0);
  EXPECT_DOUBLE_EQ(velocity.second, 0.0);
}

TEST(ControlCoreTest, StopsOnceInsideTheGoalTolerance)
{
  const auto path = makePath({{0.0, 0.0}, {1.0, 0.0}});
  const auto velocity = makeControl().computeVelocity(path, 0.9, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(velocity.first, 0.0);
  EXPECT_DOUBLE_EQ(velocity.second, 0.0);
}

TEST(ControlCoreTest, DrivesStraightAtAPointDeadAhead)
{
  const auto velocity = makeControl().computeVelocity(straightPath(), 0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(velocity.first, kMaxLinear);
  EXPECT_NEAR(velocity.second, 0.0, 1e-9);
}

TEST(ControlCoreTest, TurnsLeftForAPathBearingLeft)
{
  const auto path = makePath({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}});
  const auto velocity = makeControl().computeVelocity(path, 0.0, 0.0, 0.0);

  EXPECT_GT(velocity.first, 0.0);
  EXPECT_GT(velocity.second, 0.0);
}

TEST(ControlCoreTest, TurnsRightForAPathBearingRight)
{
  const auto path = makePath({{0.0, 0.0}, {1.0, -1.0}, {2.0, -2.0}});
  const auto velocity = makeControl().computeVelocity(path, 0.0, 0.0, 0.0);

  EXPECT_GT(velocity.first, 0.0);
  EXPECT_LT(velocity.second, 0.0);
}

// Driving forward can never reach a point behind the robot, so it has to
// rotate on the spot first.
TEST(ControlCoreTest, SpinsInPlaceForAPathBehind)
{
  const auto path = makePath({{-2.0, 0.0}, {-4.0, 0.0}});
  const auto velocity = makeControl().computeVelocity(path, 0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(velocity.first, 0.0);
  EXPECT_NE(velocity.second, 0.0);
}

// The reason the lookahead search starts from the nearest waypoint rather than
// from index 0: part-way along a path, the earliest waypoints are far away and
// behind, and a front-to-back scan would aim at one and reverse into it.
TEST(ControlCoreTest, AimsForwardWhenPartWayAlongThePath)
{
  const auto velocity = makeControl().computeVelocity(straightPath(), 5.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(velocity.first, kMaxLinear);
  EXPECT_NEAR(velocity.second, 0.0, 1e-9);
}

TEST(ControlCoreTest, RespectsTheRobotsHeading)
{
  // Path runs along +x, but the robot faces +y, so the path is off to its
  // right and it must turn clockwise.
  const auto velocity = makeControl().computeVelocity(straightPath(), 0.0, 0.0, M_PI / 2.0);

  EXPECT_LT(velocity.second, 0.0);
}

TEST(ControlCoreTest, ClampsAngularVelocity)
{
  // A tight hook: curvature here far exceeds what max_angular allows.
  const auto path = makePath({{0.0, 0.0}, {0.1, 0.5}});
  const auto velocity = makeControl().computeVelocity(path, 0.0, 0.0, 0.0);

  EXPECT_LE(std::abs(velocity.second), kMaxAngular);
  EXPECT_GT(velocity.second, 0.0);
}

TEST(ControlCoreTest, NeverExceedsTheConfiguredLimits)
{
  const auto control = makeControl();

  // Sweep a ring of goals around the robot; none may exceed the limits.
  for (int degrees = 0; degrees < 360; degrees += 15) {
    const double angle = degrees * M_PI / 180.0;
    const auto path = makePath({{3.0 * std::cos(angle), 3.0 * std::sin(angle)}});
    const auto velocity = control.computeVelocity(path, 0.0, 0.0, 0.0);

    ASSERT_LE(velocity.first, kMaxLinear);
    ASSERT_GE(velocity.first, 0.0);
    ASSERT_LE(std::abs(velocity.second), kMaxAngular);
  }
}

// Holding full speed through a tight turn is what cuts the corner into the
// obstacle the planner routed around, so speed has to fall as curvature rises.
TEST(ControlCoreTest, SlowsDownForTighterTurns)
{
  const auto control = makeControl();

  const auto straight = control.computeVelocity(straightPath(), 0.0, 0.0, 0.0);
  const auto gentle = control.computeVelocity(
    makePath({{0.0, 0.0}, {2.0, 0.5}, {4.0, 1.0}}), 0.0, 0.0, 0.0);
  const auto sharp = control.computeVelocity(
    makePath({{0.0, 0.0}, {1.0, 1.5}, {1.5, 3.0}}), 0.0, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(straight.first, kMaxLinear);
  EXPECT_LT(gentle.first, straight.first);
  EXPECT_LT(sharp.first, gentle.first);
  EXPECT_GT(sharp.first, 0.0);
}
