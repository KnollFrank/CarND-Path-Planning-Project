#include "gtest/gtest.h"
#include "coords/coords2.h"

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

  // WHEN & THEN
  // EXPECT_EQ((Frenet { 0, 0 }), getFrenet2(Point { 0, 0 }, mapWaypoints));

  EXPECT_EQ((Frenet { 5, 0 }), getFrenet2(Point { 5, 0 }, mapWaypoints));
  EXPECT_EQ((Frenet { 5, -2 }), getFrenet2(Point { 5, 2 }, mapWaypoints));
  EXPECT_EQ((Frenet { 5, 2 }), getFrenet2(Point { 5, -2 }, mapWaypoints));

  EXPECT_EQ((Frenet { 10, 0 }), getFrenet2(Point { 10, 0 }, mapWaypoints));
  EXPECT_EQ((Frenet { 15, 0 }), getFrenet2(Point { 10, 5 }, mapWaypoints));
  EXPECT_EQ((Frenet { 20, 0 }), getFrenet2(Point { 10, 10 }, mapWaypoints));
  // EXPECT_EQ((Frenet { 25, 0 }), getFrenet2(Point { 5, 10 }, mapWaypoints));
  EXPECT_EQ((Frenet { 30, 0 }), getFrenet2(Point { 0, 10 }, mapWaypoints));
  // EXPECT_EQ((Frenet { 35, 0 }), getFrenet2(Point { 0, 5 }, mapWaypoints));
}
