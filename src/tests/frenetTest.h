#ifndef TESTS_FRENETTEST_H_
#define TESTS_FRENETTEST_H_

#include <coords/coordsConverter.h>
#include <coords/frenet.h>
#include <coords/waypoints.h>
#include <gtest/gtest.h>

Frenet minus(const Frenet& past, const Frenet& prev, const double len) {
  return Frenet { len - prev.s + past.s, 0 };
}

TEST(FrenetTest, test_minus) {
  // GIVEN
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);
  Frenet prev = Frenet { 6946.8994024539452, 0 };
  Frenet past = Frenet { 0.06738587146532965, 0 };

  // WHEN
  Frenet diff = ::minus(past, prev, 6947.2427832056264);

  // THEN
  expect_near(
      Frenet { 6947.2427832056264 - 6946.8994024539452 + 0.06738587146532965, 0 },
      diff, 0.001);
}

#endif /* TESTS_FRENETTEST_H_ */
