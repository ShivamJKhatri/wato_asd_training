#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "map_memory_core.hpp"

namespace
{

constexpr int8_t LETHAL = 100;

// A 30 m x 30 m world at 0.2 m per cell, anchored at the world origin.
robot::MapMemoryCore makeMap()
{
  robot::MapMemoryCore map(rclcpp::get_logger("map_memory_test"));
  map.initGrid(0.2, 150, 150, -15.0, -15.0);
  return map;
}

// A costmap centred on the robot: `size` cells square at 0.1 m, all UNKNOWN
// until a caller fills something in.
nav_msgs::msg::OccupancyGrid makeCostmap(int size = 40)
{
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.info.resolution = 0.1;
  costmap.info.width = size;
  costmap.info.height = size;
  costmap.info.origin.position.x = -(size * 0.1) / 2.0;
  costmap.info.origin.position.y = -(size * 0.1) / 2.0;
  costmap.data.assign(size * size, robot::MapMemoryCore::UNKNOWN);
  return costmap;
}

struct LocalPoint {
  double x;
  double y;
};

// Writes `value` into the costmap cell containing (x, y) and returns that
// cell's centre.
//
// Returning the centre matters: a point on a cell boundary does not always
// round the way you would expect, because `resolution` is a float32 in the
// message. Tests predict where a cell lands from this, never from the point
// they asked for.
LocalPoint setLocal(nav_msgs::msg::OccupancyGrid & costmap, double x, double y, int8_t value)
{
  const double resolution = costmap.info.resolution;
  const double origin_x = costmap.info.origin.position.x;
  const double origin_y = costmap.info.origin.position.y;

  const int gx = static_cast<int>(std::floor((x - origin_x) / resolution));
  const int gy = static_cast<int>(std::floor((y - origin_y) / resolution));

  EXPECT_GE(gx, 0);
  EXPECT_GE(gy, 0);
  EXPECT_LT(gx, static_cast<int>(costmap.info.width));
  EXPECT_LT(gy, static_cast<int>(costmap.info.height));

  costmap.data[gy * costmap.info.width + gx] = value;

  return {origin_x + (gx + 0.5) * resolution, origin_y + (gy + 0.5) * resolution};
}

// Fills every costmap cell within `half_size` metres of (x, y). Used to blank
// a region wide enough to cover a whole map cell regardless of alignment.
void fillLocalBlock(nav_msgs::msg::OccupancyGrid & costmap, double x, double y,
                    double half_size, int8_t value)
{
  const double resolution = costmap.info.resolution;
  const double origin_x = costmap.info.origin.position.x;
  const double origin_y = costmap.info.origin.position.y;

  for (uint32_t gy = 0; gy < costmap.info.height; ++gy) {
    const double cell_y = origin_y + (gy + 0.5) * resolution;
    for (uint32_t gx = 0; gx < costmap.info.width; ++gx) {
      const double cell_x = origin_x + (gx + 0.5) * resolution;
      if (std::abs(cell_x - x) <= half_size && std::abs(cell_y - y) <= half_size) {
        costmap.data[gy * costmap.info.width + gx] = value;
      }
    }
  }
}

int8_t cellAt(const robot::MapMemoryCore & map, double x, double y)
{
  int gx = 0;
  int gy = 0;
  EXPECT_TRUE(map.worldToGrid(x, y, gx, gy));
  return map.data()[gy * map.width() + gx];
}

}  // namespace

// The whole point of starting FREE rather than UNKNOWN: /map never renders an
// unexplored region, at any moment of the run.
TEST(MapMemoryCoreTest, StartsFreeAndNeverHoldsUnknown)
{
  auto map = makeMap();

  for (const int8_t value : map.data()) {
    ASSERT_EQ(value, robot::MapMemoryCore::FREE);
  }
}

TEST(MapMemoryCoreTest, MapsWorldPointsToCells)
{
  auto map = makeMap();

  int gx = 0;
  int gy = 0;
  ASSERT_TRUE(map.worldToGrid(0.0, 0.0, gx, gy));
  EXPECT_EQ(gx, 75);
  EXPECT_EQ(gy, 75);

  ASSERT_TRUE(map.worldToGrid(-14.9, -14.9, gx, gy));
  EXPECT_EQ(gx, 0);
  EXPECT_EQ(gy, 0);

  EXPECT_FALSE(map.worldToGrid(15.0, 0.0, gx, gy));
  EXPECT_FALSE(map.worldToGrid(0.0, -15.1, gx, gy));
}

TEST(MapMemoryCoreTest, PlacesCostmapCellsWithNoRotation)
{
  auto map = makeMap();
  auto costmap = makeCostmap();

  // A cell roughly 1 m ahead of a robot sitting at (5, 5) facing +x.
  const LocalPoint cell = setLocal(costmap, 1.0, 0.0, LETHAL);
  map.integrate(costmap, 5.0, 5.0, 0.0);

  EXPECT_EQ(cellAt(map, 5.0 + cell.x, 5.0 + cell.y), LETHAL);
  EXPECT_EQ(cellAt(map, 5.0 + cell.x, 5.0 + cell.y + 1.0), robot::MapMemoryCore::FREE);
}

// The reason orientation is in the transform at all: the costmap is expressed
// in the robot's frame, so it turns as the robot turns.
TEST(MapMemoryCoreTest, RotatesCostmapByRobotYaw)
{
  auto map = makeMap();
  auto costmap = makeCostmap();

  const double yaw = M_PI / 2.0;  // facing +y
  const LocalPoint cell = setLocal(costmap, 1.0, 0.0, LETHAL);
  map.integrate(costmap, 0.0, 0.0, yaw);

  const double world_x = cell.x * std::cos(yaw) - cell.y * std::sin(yaw);
  const double world_y = cell.x * std::sin(yaw) + cell.y * std::cos(yaw);

  EXPECT_EQ(cellAt(map, world_x, world_y), LETHAL);

  // And specifically NOT where it would have landed unrotated.
  EXPECT_EQ(cellAt(map, cell.x, cell.y), robot::MapMemoryCore::FREE);
}

// The rule that makes the whole node work: a costmap that cannot see a cell
// must not be allowed to speak for it.
TEST(MapMemoryCoreTest, UnknownCostmapCellsRetainThePreviousValue)
{
  auto map = makeMap();

  auto seen = makeCostmap();
  const LocalPoint cell = setLocal(seen, 1.0, 0.0, LETHAL);
  map.integrate(seen, 0.0, 0.0, 0.0);
  ASSERT_EQ(cellAt(map, cell.x, cell.y), LETHAL);

  // Same pose, but now the whole scan is occluded and reports UNKNOWN.
  auto occluded = makeCostmap();
  map.integrate(occluded, 0.0, 0.0, 0.0);

  EXPECT_EQ(cellAt(map, cell.x, cell.y), LETHAL);
}

// The failure this design exists to avoid: driving past an obstacle must not
// erase it, or the planner will route straight back through it.
TEST(MapMemoryCoreTest, DoesNotEraseAnObstacleThatFallsOutOfView)
{
  auto map = makeMap();

  auto seen = makeCostmap();
  const LocalPoint cell = setLocal(seen, 1.0, 0.0, LETHAL);
  map.integrate(seen, 0.0, 0.0, 0.0);
  ASSERT_EQ(cellAt(map, cell.x, cell.y), LETHAL);

  // Robot drives 2 m on. Everything it can now see reads free -- including the
  // patch the obstacle occupies, which is behind it and no longer detected.
  auto later = makeCostmap();
  std::fill(later.data.begin(), later.data.end(), robot::MapMemoryCore::FREE);
  map.integrate(later, 2.0, 0.0, 0.0);

  EXPECT_EQ(cellAt(map, cell.x, cell.y), LETHAL);
}

// A later costmap reporting free must NOT clear a known obstacle: the costmap
// reports free for anything it is not currently hitting, occluded obstacles
// included, so free is not evidence of absence.
TEST(MapMemoryCoreTest, FreeDoesNotClearAKnownObstacle)
{
  auto map = makeMap();

  auto first = makeCostmap();
  const LocalPoint cell = setLocal(first, 1.0, 0.0, LETHAL);
  map.integrate(first, 0.0, 0.0, 0.0);
  ASSERT_EQ(cellAt(map, cell.x, cell.y), LETHAL);

  auto second = makeCostmap();
  fillLocalBlock(second, cell.x, cell.y, 0.35, robot::MapMemoryCore::FREE);
  map.integrate(second, 0.0, 0.0, 0.0);

  EXPECT_EQ(cellAt(map, cell.x, cell.y), LETHAL);
}

// Free space still fills in where the map knew nothing worse.
TEST(MapMemoryCoreTest, FreeCellsStillRecordWhereNothingIsKnown)
{
  auto map = makeMap();
  auto costmap = makeCostmap();
  const LocalPoint cell = setLocal(costmap, 1.0, 0.0, 40);
  map.integrate(costmap, 0.0, 0.0, 0.0);

  EXPECT_EQ(cellAt(map, cell.x, cell.y), 40);
}

// Four costmap cells land in each map cell, so the most dangerous must win
// rather than whichever the loop happened to reach last.
TEST(MapMemoryCoreTest, TakesTheHighestCostWhenCellsCollide)
{
  auto map = makeMap();
  auto costmap = makeCostmap();

  // These four 0.1 m cells all fall inside one 0.2 m map cell.
  setLocal(costmap, 1.05, 0.05, 10);
  const LocalPoint lethal = setLocal(costmap, 1.15, 0.05, LETHAL);
  setLocal(costmap, 1.05, 0.15, 10);
  setLocal(costmap, 1.15, 0.15, 10);

  map.integrate(costmap, 0.0, 0.0, 0.0);

  EXPECT_EQ(cellAt(map, lethal.x, lethal.y), LETHAL);
}

TEST(MapMemoryCoreTest, IgnoresCostmapCellsOutsideTheMap)
{
  auto map = makeMap();
  auto costmap = makeCostmap();
  setLocal(costmap, 1.0, 0.0, LETHAL);

  // Robot parked outside the map entirely; must not crash or corrupt memory.
  map.integrate(costmap, 100.0, 100.0, 0.0);

  for (const int8_t value : map.data()) {
    ASSERT_EQ(value, robot::MapMemoryCore::FREE);
  }
}
