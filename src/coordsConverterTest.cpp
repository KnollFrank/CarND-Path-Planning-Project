#include "gtest/gtest.h"

#include "coords/coordsConverter.h"

TEST(CoordsConverterTest, should_get_frenet) {
  // GIVEN
  MapWaypoints map_waypoints;

  map_waypoints.map_waypoints.push_back(Point { 0, 5 });
  map_waypoints.map_outwards.push_back(Point { -1, 0 });

  map_waypoints.map_waypoints.push_back(Point { 5, 0 });
  map_waypoints.map_outwards.push_back(Point { 0, -1 });

  map_waypoints.map_waypoints.push_back(Point { 10, 0 });
  map_waypoints.map_outwards.push_back(Point { 0, -1 });

  map_waypoints.map_waypoints.push_back(Point { 10, 10 });
  map_waypoints.map_outwards.push_back(Point { 0, 1 });

  map_waypoints.map_waypoints.push_back(Point { 0, 10 });
  map_waypoints.map_outwards.push_back(Point { 0, 1 });

  CoordsConverter coordsConverter(map_waypoints);

  // WHEN & THEN
  EXPECT_EQ((Frenet { sqrt(50) + 2, -1 }), coordsConverter.getFrenet(Point { 7,
      1 }));
}
