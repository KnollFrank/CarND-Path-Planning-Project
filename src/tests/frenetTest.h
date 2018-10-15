#ifndef TESTS_FRENETTEST_H_
#define TESTS_FRENETTEST_H_

#include <coords/frenet.h>
#include <gtest/gtest.h>
#include <tests/gtestHelper.h>

TEST(FrenetTest, test_minusCircular1) {
  // GIVEN
  const Frenet prev = Frenet { 4, 10 };
  const Frenet past = Frenet { 1, 7 };
  const double len = 5;

  // WHEN
  Frenet diff = past.minusCircular(prev, len);

  // THEN
  expect_near(Frenet { 2, 7 - 10 }, diff, 0.001);
}

TEST(FrenetTest, test_minusCircular2) {
  // GIVEN
  const Frenet prev = Frenet { 4, 10 };
  const Frenet past = Frenet { 2, 7 };
  const double len = 5;

  // WHEN
  Frenet diff = past.minusCircular(prev, len);

  // THEN
  expect_near(Frenet { 3, 7 - 10 }, diff, 0.001);
}

TEST(FrenetTest, test_minusCircular3) {
  // GIVEN
  Frenet prev = Frenet { 10, 6 };
  Frenet past = Frenet { 15, 8 };

  // WHEN
  Frenet diff = past.minusCircular(prev, 6947.2427832056264);

  // THEN
  expect_near(past - prev, diff, 0.001);
}

#endif /* TESTS_FRENETTEST_H_ */
