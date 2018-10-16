#ifndef TESTS_EGOCARTEST_H_
#define TESTS_EGOCARTEST_H_

#include <coords/cart.h>
#include <coords/coordsConverter.h>
#include <coords/frenetCart.h>
#include <coords/waypoints.h>
#include <egoCar.h>
#include <gtest/gtest.h>

TEST(EgoCarTest, shouldGetAcceleration) {
  // GIVEN
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  EgoCar egoCar(coordsConverter);

  const double dt = 0.02;
  const Point acc { 2, 3 };
  const Point vel { 5, 7 };

  auto s = [&](const double t) {
    return acc * (0.5 * t * t) + vel * t;
  };

  auto moveCarToPointAtTimeStep = [&](const int timeStep) {
    egoCar.setPos(FrenetCart(s(timeStep * dt), coordsConverter));
  };

  moveCarToPointAtTimeStep(0);
  moveCarToPointAtTimeStep(1);
  moveCarToPointAtTimeStep(2);
  moveCarToPointAtTimeStep(3);

  // WHEN
  Point acceleration = egoCar.getAcceleration(dt);

  // THEN
  expect_near(acc, acceleration, 0.001);
}

TEST(EgoCarTest, shouldGetAcceleration2) {
  // GIVEN
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  EgoCar egoCar(coordsConverter);

  const double dt = 0.02;

  Point p1 { 910.22016538328012, 1128.7915689198328 };
  Point p2 { 910.23789922405649, 1128.791464735476 };
  Point p3 { 910.25563369260044, 1128.7913601589348 };
  Point p4 { 910.29110451250983, 1128.7911498362475 };

  for (const Point& point : { p1, p2, p3, p4 }) {
    egoCar.setPos(FrenetCart(point, coordsConverter));
  }

  // WHEN
  Point acceleration = egoCar.getAcceleration(dt);

  // THEN
  Point vel1 = (p4 - p3) / dt;
  Point vel2 = (p3 - p2) / dt;
  Point acc = (vel1 - vel2) / dt;
  expect_near(acc, acceleration, 0.001);
}

#endif /* TESTS_EGOCARTEST_H_ */
