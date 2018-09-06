#include "gtest/gtest.h"
#include "pathPlanner.h"

namespace test {

bool collision(const Points &egoPath, const vector<Vehicle> &vehicles) {
  return false;
}

EgoCar createEgoCar(Frenet pos, const MapWaypoints & map_waypoints) {
  EgoCar egoCar;
  egoCar.pos_frenet = pos;
  egoCar.pos_cart = getXY(pos.s, pos.d, map_waypoints);
  egoCar.yaw = 0;
  egoCar.speed = 0;
  return egoCar;
}

}

TEST(PathPlanningTest, should_ego_drive_in_same_lane) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel = 0;
  int lane = 1;
  Frenet pos = Frenet { 124.8336, 6 };
  EgoCar egoCar = test::createEgoCar(pos, map_waypoints);

  PreviousData previousData;
  vector<Vehicle> vehicles;

// WHEN
  Points path = createPath(refPoint, lane, map_waypoints, egoCar, previousData,
                           vehicles);

// THEN
  ASSERT_GT(path.xs.size(), 0);
  Frenet frenet = getFrenet(path.xs[0], path.ys[0], 0, map_waypoints);
  ASSERT_NEAR(6.0, frenet.d, 0.0001);
  ASSERT_FALSE(test::collision(path, vehicles));
}
