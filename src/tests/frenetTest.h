#ifndef TESTS_FRENETTEST_H_
#define TESTS_FRENETTEST_H_

#include <coords/frenet.h>
#include <gtest/gtest.h>
#include <tests/gtestHelper.h>

Frenet minusCircular(const Frenet& past, const Frenet& prev, const double len) {
  const Frenet diff = past - prev;
  return prev.s <= past.s ? diff : diff + Frenet { len, 0 };
}

TEST(FrenetTest, test_minusCircular1) {
  // GIVEN
  const Frenet prev = Frenet { 6946.8994024539452, 10 };
  const Frenet past = Frenet { 0.06738587146532965, 7 };
  const double len = 6947.2427832056264;

  // WHEN
  Frenet diff = ::minusCircular(past, prev, len);

  // THEN
  expect_near(Frenet { len - prev.s + past.s, past.d - prev.d }, diff, 0.001);
}

TEST(FrenetTest, test_minusCircular2) {
  // GIVEN
  Frenet prev = Frenet { 10, 6 };
  Frenet past = Frenet { 15, 8 };

  // WHEN
  Frenet diff = ::minusCircular(past, prev, 6947.2427832056264);

  // THEN
  expect_near(past - prev, diff, 0.001);
}

#endif /* TESTS_FRENETTEST_H_ */
