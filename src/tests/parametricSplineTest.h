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
  double length = spline.getLength();

  // Then
  EXPECT_EQ(6947, int(length));
}

void checkDistanceBetweenPointAndSpline(
    const Frenet& frenet, const ParametricSpline& spline,
    const CoordsConverter& coordsConverter) {

  // Given
  double distanceExpected = frenet.d;
  const Point& point = coordsConverter.getXY(frenet);

  // When
  double distanceActual = spline.distanceTo(point);

  // Then
  EXPECT_NEAR(distanceExpected, distanceActual, 0.001);
}

TEST(ParametricSplineTest, should_get_distanceBetweenPointAndSpline) {
  // Given
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  ParametricSpline spline(mapWaypoints.map_waypoints);
  double splineLength = spline.getLength();

  // When & Then
  for (double d : { 2.0, 6.0, 10.0 }) {
    for (double s = 0.0; s < splineLength; s += 10) {
      checkDistanceBetweenPointAndSpline(Frenet { s, d }, spline,
                                         coordsConverter);
    }
    for (double s : mapWaypoints.map_waypoints_s) {
      checkDistanceBetweenPointAndSpline(Frenet { s, d }, spline,
                                         coordsConverter);
    }
  }
}

#endif /* TESTS_PARAMETRICSPLINETEST_H_ */
