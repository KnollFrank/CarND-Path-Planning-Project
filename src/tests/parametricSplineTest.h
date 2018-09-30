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

TEST(ParametricSplineTest, should_get_distanceBetweenPointAndSpline) {
  // Given
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);

  ParametricSpline spline(mapWaypoints.map_waypoints);
  double distanceExpected = 5;
  Point point = coordsConverter.getXY(Frenet { 10, distanceExpected });

  // When
  double distanceActual = distance(point, spline);

  // Then
  EXPECT_NEAR(distanceExpected, distanceActual, 0.2);
}

#endif /* TESTS_PARAMETRICSPLINETEST_H_ */
