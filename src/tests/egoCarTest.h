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

  egoCar.setPos(FrenetCart(Point {0, 0}, coordsConverter));
  egoCar.setPos(FrenetCart(Point {10, 0}, coordsConverter));
  egoCar.setPos(FrenetCart(Point {20, 0}, coordsConverter));
  egoCar.setPos(FrenetCart(Point {30, 0}, coordsConverter));

  // WHEN
  double acceleration = egoCar.getAcceleration(1).len();

  // THEN
  ASSERT_EQ(0, acceleration);
}

#endif /* TESTS_EGOCARTEST_H_ */
