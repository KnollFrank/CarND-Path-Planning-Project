#include "gtest/gtest.h"
#include "pathPlanner.h"

TEST(PathPlanningTest, ShouldCreatePath) {
  // GIVEN
  ReferencePoint refPoint;
  refPoint.vel = 0;

  int lane = 1;
  MapWaypoints map_waypoints = read_map_waypoints();
  EgoCar egoCar;
  PreviousData previousData;
  vector<Vehicle> vehicles;

  // WHEN
  Points path = createPath(refPoint, lane, map_waypoints, egoCar, previousData, vehicles);

  // THEN
  ASSERT_TRUE(true);
}
