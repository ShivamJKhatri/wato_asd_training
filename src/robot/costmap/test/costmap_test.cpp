#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "costmap_core.hpp"

namespace
{

// Matches the node's defaults: a 20 m x 20 m window at 0.1 m per cell, so the
// robot sits at cell (100, 100) and the grid spans -10 m to +10 m.
robot::CostmapCore makeCostmap(double robot_radius = 0.7, double inflation_radius = 1.5)
{
  robot::CostmapCore costmap(rclcpp::get_logger("costmap_test"));
  costmap.initGrid(0.1, 200, 200);
  costmap.setInflation(robot_radius, inflation_radius);
  return costmap;
}

int8_t cellAt(const robot::CostmapCore & costmap, double x, double y)
{
  int grid_x = 0;
  int grid_y = 0;
  EXPECT_TRUE(costmap.worldToGrid(x, y, grid_x, grid_y));
  return costmap.data()[grid_y * costmap.width() + grid_x];
}

}  // namespace

TEST(CostmapCoreTest, RobotSitsAtTheGridCentre)
{
  auto costmap = makeCostmap();

  int x = -1;
  int y = -1;
  ASSERT_TRUE(costmap.worldToGrid(0.0, 0.0, x, y));
  EXPECT_EQ(x, 100);
  EXPECT_EQ(y, 100);
}

TEST(CostmapCoreTest, ConvertsMetresToCells)
{
  auto costmap = makeCostmap();

  int x = 0;
  int y = 0;
  ASSERT_TRUE(costmap.worldToGrid(1.0, 2.0, x, y));
  EXPECT_EQ(x, 110);
  EXPECT_EQ(y, 120);
}

TEST(CostmapCoreTest, RejectsPointsOutsideTheWindow)
{
  auto costmap = makeCostmap();

  int x = 0;
  int y = 0;
  EXPECT_FALSE(costmap.worldToGrid(10.0, 0.0, x, y));  // exactly on the far edge
  EXPECT_FALSE(costmap.worldToGrid(0.0, 15.0, x, y));
  EXPECT_FALSE(costmap.worldToGrid(-20.0, 0.0, x, y));
}

// The reason worldToGrid floors instead of casting: a plain cast truncates
// toward zero, so this point would land in cell 0 and be treated as inside.
TEST(CostmapCoreTest, RejectsPointsJustOutsideTheLowerEdge)
{
  auto costmap = makeCostmap();

  int x = 0;
  int y = 0;
  EXPECT_FALSE(costmap.worldToGrid(-10.05, 0.0, x, y));
  EXPECT_FALSE(costmap.worldToGrid(0.0, -10.05, x, y));
}

// The costmap is optimistic by design: space no beam stopped in reads free.
// It must never publish UNKNOWN, which is what keeps grey off every layer.
TEST(CostmapCoreTest, StartsEntirelyFree)
{
  auto costmap = makeCostmap();

  EXPECT_EQ(cellAt(costmap, 0.0, 0.0), robot::CostmapCore::FREE);
  EXPECT_EQ(cellAt(costmap, 5.0, -5.0), robot::CostmapCore::FREE);
}

TEST(CostmapCoreTest, NeverPublishesUnknownCells)
{
  auto costmap = makeCostmap();
  costmap.markObstacle(2.0, 0.0);
  costmap.inflate();

  for (const int8_t value : costmap.data()) {
    ASSERT_NE(value, robot::CostmapCore::UNKNOWN);
  }
}

TEST(CostmapCoreTest, MarksTheCellContainingTheObstacle)
{
  auto costmap = makeCostmap();
  costmap.markObstacle(1.0, 2.0);

  EXPECT_EQ(cellAt(costmap, 1.0, 2.0), robot::CostmapCore::LETHAL);
  EXPECT_EQ(cellAt(costmap, -5.0, -5.0), robot::CostmapCore::FREE);
}

TEST(CostmapCoreTest, IgnoresObstaclesOutsideTheWindow)
{
  auto costmap = makeCostmap();

  // Must not crash or corrupt memory: a 19 m return is in range for the sensor
  // but well outside this window.
  costmap.markObstacle(19.0, 0.0);

  EXPECT_EQ(cellAt(costmap, 0.0, 0.0), robot::CostmapCore::FREE);
}

TEST(CostmapCoreTest, ResetClearsBackToFree)
{
  auto costmap = makeCostmap();
  costmap.markObstacle(1.0, 2.0);
  ASSERT_EQ(cellAt(costmap, 1.0, 2.0), robot::CostmapCore::LETHAL);

  costmap.reset();

  EXPECT_EQ(cellAt(costmap, 1.0, 2.0), robot::CostmapCore::FREE);
}

TEST(CostmapCoreTest, InflationFadesWithDistance)
{
  auto costmap = makeCostmap(0.7, 1.5);
  costmap.markObstacle(2.0, 0.0);
  costmap.inflate();

  const int8_t near_cost = cellAt(costmap, 2.0, 0.4);   // inside robot_radius
  const int8_t far_cost = cellAt(costmap, 2.0, 1.2);    // out in the fade

  EXPECT_EQ(cellAt(costmap, 2.0, 0.0), robot::CostmapCore::LETHAL);
  EXPECT_EQ(near_cost, robot::CostmapCore::INSCRIBED);
  EXPECT_GT(far_cost, 0);
  EXPECT_LT(far_cost, near_cost);
}

TEST(CostmapCoreTest, InflationStopsAtTheRadius)
{
  auto costmap = makeCostmap(0.7, 1.5);
  costmap.markObstacle(2.0, 0.0);
  costmap.inflate();

  // 4.0 m is 2.0 m past the obstacle, outside the 1.5 m disc.
  EXPECT_EQ(cellAt(costmap, 4.0, 0.0), robot::CostmapCore::FREE);
}

TEST(CostmapCoreTest, InflationNeverLowersAnExistingCost)
{
  auto costmap = makeCostmap(0.7, 1.5);

  // Two obstacles 1 m apart: the cells between them are inside both discs and
  // must end up with the higher of the two costs.
  costmap.markObstacle(2.0, 0.0);
  costmap.markObstacle(3.0, 0.0);
  costmap.inflate();

  EXPECT_EQ(cellAt(costmap, 2.5, 0.0), robot::CostmapCore::INSCRIBED);
}

// Inflation reads the grid while writing to it, so a freshly stamped cost must
// not be mistaken for an obstacle and seed a second round of spreading.
TEST(CostmapCoreTest, InflationDoesNotCascade)
{
  auto costmap = makeCostmap(0.7, 1.5);
  costmap.markObstacle(0.0, 0.0);
  costmap.inflate();

  // 2.0 m out is well past the 1.5 m disc; a cascade would have reached it.
  EXPECT_EQ(cellAt(costmap, 2.0, 0.0), robot::CostmapCore::FREE);
  EXPECT_EQ(cellAt(costmap, 0.0, -2.0), robot::CostmapCore::FREE);
}
