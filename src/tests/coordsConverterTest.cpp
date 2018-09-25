#include "gtest/gtest.h"

#include "../coords/coordsConverter.h"
#include "../parametricSpline.h"

#define GTEST_COUT std::cerr

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

  auto test_convert = [&](const Point& point, const Frenet& frenet) {
    expect_near(frenet, coordsConverter.getFrenet(point));
    expect_near(point, coordsConverter.getXY(frenet));
  };

  // WHEN & THEN
  double s1 = 5;
  double s2 = sqrt(50);

  test_convert(Point { 0, 4 }, Frenet { s1 + 1 / sqrt(2), 1 / sqrt(2) });
  test_convert(Point { 4, 0 }, Frenet { s1 + s2 - 1 / sqrt(2), 1 / sqrt(2) });
  test_convert(Point { 9, 0.5 }, Frenet { s1 + s2 + 4, -0.5 });
}

void print_array(string name, vector<double> xs) {
  GTEST_COUT<< name << " = [";
  for (int i = 0; i < xs.size(); i++) {
    GTEST_COUT<< xs[i] << ", " << endl;
  }
  GTEST_COUT<< "]";
}

TEST(CoordsConverterTest, should_convert2) {
  pspline2interpolant p;
  real_2d_array xy("[[0, 0], [10, 0], [10, 10], [0, 10]]");
  buildPeriodicParametricSpline(xy, SplineType::CatmullRom, ParameterizationType::uniform, p);
  double x;
  double y;
  vector<double> xs;
  vector<double> ys;

  for (double t = 0.0; t < 1.0; t += 0.01) {
    pspline2calc(p, t, x, y);
    // GTEST_COUT << "p(" << t << ") = (" << x << ", " << y << ")" << endl;
    // GTEST_COUT << x << " " << y << endl;
    xs.push_back(x);
    ys.push_back(y);
  }

  print_array("x", xs);
  GTEST_COUT<< endl;
  print_array("y", ys);

  double arclength = pspline2arclength(p, 0, 1);
  EXPECT_NEAR(40, arclength, 0.00001);
}
