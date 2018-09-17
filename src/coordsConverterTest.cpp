#include "gtest/gtest.h"

#include "coords/coordsConverter.h"

void expect_near(const Frenet& expected, const Frenet& actual) {
  const double abs_error = 0.00001;
  EXPECT_NEAR(expected.s, actual.s, abs_error);
  EXPECT_NEAR(expected.d, actual.d, abs_error);
}

void expect_near(const Point& expected, const Point& actual) {
  const double abs_error = 0.00001;
  EXPECT_NEAR(expected.x, actual.x, abs_error);
  EXPECT_NEAR(expected.y, actual.y, abs_error);
}

void test_convert(const CoordsConverter &coordsConverter, const Point& point,
                  const Frenet& frenet) {
  expect_near(frenet, coordsConverter.getFrenet(point));
  expect_near(point, coordsConverter.getXY(frenet));
}

TEST(CoordsConverterTest, should_convert) {
  // GIVEN
  MapWaypoints map_waypoints;

  map_waypoints.map_waypoints.push_back(Point { 0, 10 });
  map_waypoints.map_outwards.push_back(Point { -1, 1 });

  map_waypoints.map_waypoints.push_back(Point { 0, 5 });
  map_waypoints.map_outwards.push_back(Point { -1, 0 });

  map_waypoints.map_waypoints.push_back(Point { 5, 0 });
  map_waypoints.map_outwards.push_back(Point { -1, -1 });

  map_waypoints.map_waypoints.push_back(Point { 10, 0 });
  map_waypoints.map_outwards.push_back(Point { 1, -1 });

  map_waypoints.map_waypoints.push_back(Point { 10, 10 });
  map_waypoints.map_outwards.push_back(Point { 1, 1 });

  for (int i = 0; i < map_waypoints.map_waypoints.size(); i++) {
    map_waypoints.map_waypoints_s.push_back(
        map_waypoints.getDistanceFromWaypointZeroToWaypoint(i));
  }

  CoordsConverter coordsConverter(map_waypoints);

  // WHEN & THEN
  double s1 = 5;
  double s2 = sqrt(50);

  test_convert(coordsConverter, Point { 0, 4 },
               Frenet { s1 + 1 / sqrt(2), 1 / sqrt(2) });

  test_convert(coordsConverter, Point { 4, 0 },
               Frenet { s1 + s2 - 1 / sqrt(2), 1 / sqrt(2) });

  test_convert(coordsConverter, Point { 9, 0.5 }, Frenet { s1 + s2 + 4, -0.5 });
}
