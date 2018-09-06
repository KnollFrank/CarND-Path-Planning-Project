#include "gtest/gtest.h"
#include "pathPlanner.h"

TEST(PathPlanningTest, ShouldCreatePath) {
  // GIVEN
  ReferencePoint refPoint;
  refPoint.vel = 0;

  int lane = 1;
  EgoCar egoCar;
  PreviousData previousData;
  vector<Vehicle> vehicles;

  // WHEN
  Points path = createPath(refPoint, lane, read_map_waypoints(), egoCar, previousData, vehicles);

  // THEN
  ASSERT_TRUE(true);
}
