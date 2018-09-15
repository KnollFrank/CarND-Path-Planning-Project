#include "gtest/gtest.h"

#include "coords/coordsConverter.h"

void expect_near(const Frenet& expected, const Frenet& actual,
                 double abs_error) {
  EXPECT_NEAR(expected.s, actual.s, abs_error);
  EXPECT_NEAR(expected.d, actual.d, abs_error);
}

TEST(CoordsConverterTest, should_get_frenet) {
  // GIVEN
  MapWaypoints map_waypoints;

  map_waypoints.map_waypoints.push_back(Point { 0, 10 });
  map_waypoints.map_outwards.push_back(Point { 0, 1 });

  map_waypoints.map_waypoints.push_back(Point { 0, 5 });
  map_waypoints.map_outwards.push_back(Point { -1, 0 });

  map_waypoints.map_waypoints.push_back(Point { 5, 0 });
  map_waypoints.map_outwards.push_back(Point { 0, -1 });

  map_waypoints.map_waypoints.push_back(Point { 10, 0 });
  map_waypoints.map_outwards.push_back(Point { 0, -1 });

  map_waypoints.map_waypoints.push_back(Point { 10, 10 });
  map_waypoints.map_outwards.push_back(Point { 0, 1 });

  CoordsConverter coordsConverter(map_waypoints);

  // WHEN & THEN
  expect_near((Frenet { 5 + 1 / sqrt(2), 1 / sqrt(2) }),
              coordsConverter.getFrenet(Point { 0, 4 }), 0.00001);

  expect_near((Frenet { 5 + sqrt(50) + 2, -1 }),
              coordsConverter.getFrenet(Point { 7, 1 }), 0.00001);
}
