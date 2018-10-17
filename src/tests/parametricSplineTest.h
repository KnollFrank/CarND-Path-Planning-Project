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
    // TODO: lasse die Schleife weiter als splineLength laufen z.B. bis splineLength + 10 und teste dann, ob die S-Koordinaten modulo Splinelänge übereinstimmen (verwenden dazu fmod).
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

void print_spline(const ParametricSpline& spline, const string& varName,
                  const double sStart, const double sEnd, const double sInc,
                  auto getValueFromPoint) {
  GTEST_COUT<< varName << " = [";
  for (double s = sStart; s <= sEnd; s += sInc) {
    Point point = spline(spline.toSplineParameter(s));
    GTEST_COUT << getValueFromPoint(point) << ", ";
  }
  GTEST_COUT << "]" << endl;
}

void print_arclength(const CoordsConverter& coordsConverter) {
  const vector<Point>& map_waypoints = coordsConverter.getMapWaypoints()
      .map_waypoints;
  const vector<double>& map_waypoints_s = coordsConverter.getMapWaypoints()
      .map_waypoints_s;
  ParametricSpline* spline = coordsConverter.getSpline();

  for (int i = 0; i < map_waypoints.size(); i++) {
    GTEST_COUT<< "line, spline: " << map_waypoints_s[i] << ", " << spline->getFrenet(map_waypoints[i]).s << endl;
  }
}

TEST(ParametricSplineTest, print_spline_for_display) {
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  ParametricSpline spline(mapWaypoints.map_waypoints);

  const double sStart = 0;
  const double sEnd = spline.getLength();
  const double sInc = 2;
  print_spline(spline, "x", sStart, sEnd, sInc,
               [](const Point& point) {return point.x;});
  print_spline(spline, "y", sStart, sEnd, sInc,
               [](const Point& point) {return point.y;});
  print_arclength(coordsConverter);
}

#endif /* TESTS_PARAMETRICSPLINETEST_H_ */
