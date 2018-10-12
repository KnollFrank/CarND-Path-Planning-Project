#ifndef TESTS_PARAMETRICSPLINETEST_H_
#define TESTS_PARAMETRICSPLINETEST_H_

#include "../parametricSpline.h"

#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <vector>

#include "../coords/cart.h"
#include "../coords/coordsConverter.h"
#include "../coords/frenet.h"
#include "../coords/waypoints.h"
#include "gtestHelper.h"

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

void check_getFrenet(const Frenet& frenet, const ParametricSpline& spline,
                     const CoordsConverter& coordsConverter) {

  // Given
  const Point& point = coordsConverter.getXY(frenet);

  // When
  Frenet frenetActual = spline.getFrenet(point);

  // Then
  expect_near(frenet, frenetActual, 0.001);
}

TEST(ParametricSplineTest, should_getFrenet) {
  // Given
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  ParametricSpline spline(mapWaypoints.map_waypoints);
  double splineLength = spline.getLength();  // = 6947.2427832056264

  // When & Then
  for (double d : { 2.0, 6.0, 10.0 }) {
    // FIXME: double s = 0 ergibt Fehler!
    for (double s = 1; s < splineLength; s += 10) {
      check_getFrenet(Frenet { s, d }, spline, coordsConverter);
    }
    // FIXME: int i = 0 (d.h. mapWaypoints.map_waypoints_s[0] == 0) ergibt Fehler!
    for (int i = 1; i < mapWaypoints.map_waypoints_s.size(); i++) {
      check_getFrenet(Frenet { mapWaypoints.map_waypoints_s[i], d }, spline,
                      coordsConverter);
    }
  }
}

TEST(ParametricSplineTest, should_getFrenet_for_periodic_param) {
  // Given
  MapWaypoints mapWaypoints = MapWaypoints::load();
  ParametricSpline spline(mapWaypoints.map_waypoints);
  const double abs_error = 0.0001;

  // When & Then
  {
    double delta = 0.1;
    expect_near(spline(0 + delta), spline(1 + delta), abs_error);
  }

  {
    double delta = spline.getLength() / 4;
    expect_near(spline(spline.toSplineParameter(0 + delta)),
                spline(spline.toSplineParameter(spline.getLength() + delta)),
                abs_error);
  }
}

TEST(ParametricSplineTest, print_spline_for_display) {
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  ParametricSpline spline(mapWaypoints.map_waypoints);

  GTEST_COUT << "x = [";
  for (double s = 0; s < spline.getLength(); s += 10) {
    Point point = spline(spline.toSplineParameter(s));
    GTEST_COUT << point.x << ", ";
  }
  GTEST_COUT << "]" << endl;

  GTEST_COUT << "y = [";
  for (double s = 0; s < spline.getLength(); s += 10) {
    Point point = spline(spline.toSplineParameter(s));
    GTEST_COUT << point.y << ", ";
  }
  GTEST_COUT << "]";
}

#endif /* TESTS_PARAMETRICSPLINETEST_H_ */
