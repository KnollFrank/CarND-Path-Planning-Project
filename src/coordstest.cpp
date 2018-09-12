#include "coords/coords.h"

#include "gtest/gtest.h"

TEST(CoordsTest, should_get_frenet) {
  // GIVEN
  MapWaypoints mapWaypoints;
  mapWaypoints.map_waypoints.push_back(Point { 0, 0 });
  mapWaypoints.map_waypoints.push_back(Point { 10, 0 });
  mapWaypoints.map_waypoints.push_back(Point { 10, 10 });
  mapWaypoints.map_waypoints.push_back(Point { 0, 10 });

  mapWaypoints.map_waypoints_s.push_back(0);
  mapWaypoints.map_waypoints_s.push_back(10);
  mapWaypoints.map_waypoints_s.push_back(20);
  mapWaypoints.map_waypoints_s.push_back(30);

  // WHEN
  Frenet frenet = getFrenet2(Point {5, 0}, mapWaypoints);

  // THEN
  ASSERT_EQ((Frenet {10, 6}), frenet);
}
