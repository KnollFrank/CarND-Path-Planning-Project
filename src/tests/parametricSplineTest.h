#ifndef TESTS_PARAMETRICSPLINETEST_H_
#define TESTS_PARAMETRICSPLINETEST_H_

#include "../parametricSpline.h"

#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <vector>

#include "../coords/waypoints.h"

#define GTEST_COUT std::cerr

void print_array(string name, vector<double> xs) {
  GTEST_COUT<< name << " = [";
  for (int i = 0; i < xs.size(); i++) {
    GTEST_COUT<< xs[i] << ", " << endl;
  }
  GTEST_COUT<< "]";
}

TEST(ParametricSplineTest, should_get_length) {
  // Given
  MapWaypoints mapWaypoints = MapWaypoints::load();
  ParametricSpline spline(mapWaypoints.map_waypoints);

  // When
  double length = spline.length();

  // Then
  EXPECT_EQ(6947, int(length));
}

void check_distanceTo(const Frenet& frenet,
                      const CoordsConverter& coordsConverter,
                      const ParametricSpline& spline) {

  // Given
  const Point& point = coordsConverter.getXY(frenet);

  // When
  double distance = spline.distanceTo(point);

  // Then
  EXPECT_NEAR(frenet.d, distance, 0.001);
}

TEST(ParametricSplineTest, should_get_distanceBetweenPointAndSpline) {
  // Given
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  ParametricSpline spline(mapWaypoints.map_waypoints);

  check_distanceTo(Frenet { 10, 5 }, coordsConverter, spline);
}

TEST(ParametricSplineTest, should_get_distanceBetweenPointAndSpline2) {
  // Given
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  ParametricSpline spline(mapWaypoints.map_waypoints);

  check_distanceTo(Frenet { 124.834, 5 }, coordsConverter, spline);
}

#endif /* TESTS_PARAMETRICSPLINETEST_H_ */
