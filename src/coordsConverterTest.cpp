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

  Frenet frenet1 = Frenet { s1 + 1 / sqrt(2), 1 / sqrt(2) };
  Point point1 = Point { 0, 4 };
  expect_near(frenet1, coordsConverter.getFrenet(point1));
  expect_near(point1, coordsConverter.getXY(frenet1));

  Frenet frenet2 = Frenet { s1 + s2 - 1 / sqrt(2), 1 / sqrt(2) };
  Point point2 = Point { 4, 0 };
  expect_near(frenet2, coordsConverter.getFrenet(point2));
  expect_near(point2, coordsConverter.getXY(frenet2));

  Frenet frenet3 = Frenet { s1 + s2 + 4, -0.5 };
  Point point3 = Point { 9, 0.5 };
  expect_near(frenet3, coordsConverter.getFrenet(point3));
  expect_near(point3, coordsConverter.getXY(frenet3));
}
