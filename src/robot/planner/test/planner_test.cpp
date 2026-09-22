#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

#include "planner_core.hpp"

namespace
{

constexpr int8_t LETHAL = 100;

robot::PlannerCore makePlanner()
{
  return robot::PlannerCore(rclcpp::get_logger("planner_test"));
}

// A 20 x 20 m map at 1 m per cell with its corner at the origin, so a world
// coordinate of 3.5 is cell 3 and nothing needs converting by hand.
nav_msgs::msg::OccupancyGrid makeMap(int size = 20)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "sim_world";
  map.info.resolution = 1.0;
  map.info.width = size;
  map.info.height = size;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.data.assign(size * size, 0);
  return map;
}

void setCell(nav_msgs::msg::OccupancyGrid & map, int x, int y, int8_t value)
{
  map.data[y * map.info.width + x] = value;
}

int8_t cellAt(const nav_msgs::msg::OccupancyGrid & map, double x, double y)
{
  const int gx = static_cast<int>(std::floor(x / map.info.resolution));
  const int gy = static_cast<int>(std::floor(y / map.info.resolution));
  return map.data[gy * map.info.width + gx];
}

// A wall across the middle of the map with one gap, so a route exists but only
// through the gap.
void addWallWithGap(nav_msgs::msg::OccupancyGrid & map, int x, int gap_y)
{
  for (int y = 0; y < static_cast<int>(map.info.height); ++y) {
    if (y != gap_y) {
      setCell(map, x, y, LETHAL);
    }
  }
}

}  // namespace

TEST(PlannerCoreTest, FindsAStraightRunAcrossAnEmptyMap)
{
  const auto path = makePlanner().plan(makeMap(), 0.5, 0.5, 15.5, 0.5);

  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().first, 0.5, 1e-9);
  EXPECT_NEAR(path.front().second, 0.5, 1e-9);
  EXPECT_NEAR(path.back().first, 15.5, 1e-9);
  EXPECT_NEAR(path.back().second, 0.5, 1e-9);

  // 16 cells apart in a straight line, so 16 waypoints and no detour.
  EXPECT_EQ(path.size(), 16u);
}

TEST(PlannerCoreTest, RoutesThroughTheGapInAWall)
{
  auto map = makeMap();
  addWallWithGap(map, 10, 18);

  const auto path = makePlanner().plan(map, 0.5, 0.5, 19.5, 0.5);

  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.back().first, 19.5, 1e-9);

  // It has to climb to the gap and come back, so it cannot be the direct run.
  EXPECT_GT(path.size(), 20u);
}

// The point of the whole node: a path that clips an obstacle is worse than no
// path at all.
TEST(PlannerCoreTest, NeverRoutesThroughABlockedCell)
{
  auto map = makeMap();
  addWallWithGap(map, 10, 18);

  const auto path = makePlanner().plan(map, 0.5, 0.5, 19.5, 0.5);
  ASSERT_FALSE(path.empty());

  for (const auto & point : path) {
    ASSERT_LT(cellAt(map, point.first, point.second), robot::PlannerCore::BLOCKED);
  }
}

TEST(PlannerCoreTest, ReturnsNothingWhenTheGoalIsWalledOff)
{
  auto map = makeMap();
  for (int y = 0; y < 20; ++y) {  // solid wall, no gap
    setCell(map, 10, y, LETHAL);
  }

  EXPECT_TRUE(makePlanner().plan(map, 0.5, 0.5, 19.5, 0.5).empty());
}

// Clicking near a wall is easy to do, and refusing outright made the robot
// stop dead. Plan to the closest driveable cell instead.
TEST(PlannerCoreTest, SnapsAGoalInsideAnObstacleToTheNearestFreeCell)
{
  auto map = makeMap();
  setCell(map, 15, 15, LETHAL);

  const auto path = makePlanner().plan(map, 0.5, 0.5, 15.5, 15.5);

  ASSERT_FALSE(path.empty());
  EXPECT_LT(cellAt(map, path.back().first, path.back().second),
            robot::PlannerCore::BLOCKED);
  // ...and it snaps somewhere adjacent, not to the far side of the map.
  EXPECT_LT(std::hypot(path.back().first - 15.5, path.back().second - 15.5), 3.0);
}

// Without this the robot's own safety margin walls it in: every neighbour is
// blocked, no plan exists, and it sits there forever.
TEST(PlannerCoreTest, PlansOutWhenTheRobotStartsInsideInflation)
{
  auto map = makeMap();
  for (int y = 3; y <= 7; ++y) {
    for (int x = 3; x <= 7; ++x) {
      setCell(map, x, y, robot::PlannerCore::BLOCKED);  // inflation, not lethal
    }
  }

  const auto path = makePlanner().plan(map, 5.5, 5.5, 15.5, 15.5);

  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.back().first, 15.5, 1e-9);
}

TEST(PlannerCoreTest, StillRefusesToDriveThroughAnActualObstacle)
{
  auto map = makeMap();
  for (int y = 0; y < 20; ++y) {
    setCell(map, 10, y, LETHAL);
  }

  EXPECT_TRUE(makePlanner().plan(map, 0.5, 0.5, 19.5, 0.5).empty());
}

TEST(PlannerCoreTest, ReturnsNothingWhenAnEndpointIsOffTheMap)
{
  auto map = makeMap();

  EXPECT_TRUE(makePlanner().plan(map, 0.5, 0.5, 99.0, 0.5).empty());
  EXPECT_TRUE(makePlanner().plan(map, -5.0, 0.5, 5.5, 0.5).empty());
}

TEST(PlannerCoreTest, HandlesStartEqualToGoal)
{
  const auto path = makePlanner().plan(makeMap(), 5.5, 5.5, 5.5, 5.5);

  ASSERT_EQ(path.size(), 1u);
  EXPECT_NEAR(path.front().first, 5.5, 1e-9);
}

// Inflation is passable but expensive, so the path should bulge around it
// rather than drive down the middle of the gradient.
TEST(PlannerCoreTest, PrefersOpenSpaceOverInflation)
{
  auto map = makeMap();
  for (int y = 4; y <= 6; ++y) {
    setCell(map, 10, y, 90);  // costly but under BLOCKED
  }

  const auto path = makePlanner().plan(map, 0.5, 5.5, 19.5, 5.5);
  ASSERT_FALSE(path.empty());

  for (const auto & point : path) {
    ASSERT_LT(cellAt(map, point.first, point.second), 90);
  }
}

TEST(PlannerCoreTest, ReturnsNothingForAnEmptyMap)
{
  nav_msgs::msg::OccupancyGrid empty;
  EXPECT_TRUE(makePlanner().plan(empty, 0.0, 0.0, 1.0, 1.0).empty());
}
