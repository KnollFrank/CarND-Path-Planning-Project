#ifndef TESTS_EGOCARTEST_H_
#define TESTS_EGOCARTEST_H_

#include <coords/coordsConverter.h>
#include <coords/waypoints.h>
#include <egoCar.h>
#include <gtest/gtest.h>

TEST(EgoCarTest, shouldGetAcceleration) {
  // GIVEN
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  EgoCar egoCar(coordsConverter);

  const double dt = 0.02;
  const double acc = 2;
  const double v = 5;

  auto s = [&](double t) {return 0.5 * acc * t * t + v * t;};

  egoCar.setPos(FrenetCart(Point { s(0 * dt), 0 }, coordsConverter));
  egoCar.setPos(FrenetCart(Point { s(1 * dt), 0 }, coordsConverter));
  egoCar.setPos(FrenetCart(Point { s(2 * dt), 0 }, coordsConverter));
  egoCar.setPos(FrenetCart(Point { s(3 * dt), 0 }, coordsConverter));

  // WHEN
  double acceleration = egoCar.getAcceleration(dt).len();

  // THEN
  ASSERT_NEAR(acc, acceleration, 0.001);
}

#endif /* TESTS_EGOCARTEST_H_ */
