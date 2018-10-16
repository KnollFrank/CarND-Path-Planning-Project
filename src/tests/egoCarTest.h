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

#endif /* TESTS_EGOCARTEST_H_ */
