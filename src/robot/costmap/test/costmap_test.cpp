#include <gtest/gtest.h>

#include "costmap_core.hpp"

namespace
{

// Matches the node's configuration: a 20 m x 20 m window at 0.1 m per cell, so
// the robot sits at cell (100, 100) and the grid spans -10 m to +10 m.
robot::CostmapCore makeCostmap()
{
  robot::CostmapCore costmap(rclcpp::get_logger("costmap_test"));
  costmap.initGrid(0.1, 200, 200);
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

TEST(CostmapCoreTest, MarksTheCellContainingTheObstacle)
{
  auto costmap = makeCostmap();
  costmap.markObstacle(1.0, 2.0);

  EXPECT_EQ(cellAt(costmap, 1.0, 2.0), robot::CostmapCore::LETHAL);
  EXPECT_EQ(cellAt(costmap, -1.0, -2.0), robot::CostmapCore::FREE);
}

TEST(CostmapCoreTest, IgnoresObstaclesOutsideTheWindow)
{
  auto costmap = makeCostmap();

  // Must not crash or corrupt memory: a 19 m return is in range for the sensor
  // but well outside this window.
  costmap.markObstacle(19.0, 0.0);

  EXPECT_EQ(cellAt(costmap, 0.0, 0.0), robot::CostmapCore::FREE);
}

TEST(CostmapCoreTest, ResetClearsMarkedCells)
{
  auto costmap = makeCostmap();
  costmap.markObstacle(1.0, 2.0);
  ASSERT_EQ(cellAt(costmap, 1.0, 2.0), robot::CostmapCore::LETHAL);

  costmap.reset();

  EXPECT_EQ(cellAt(costmap, 1.0, 2.0), robot::CostmapCore::FREE);
}
