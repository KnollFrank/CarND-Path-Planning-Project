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
  MapWaypoints mapWaypoints = MapWaypoints::load();
  ParametricSpline spline(mapWaypoints.map_waypoints, SplineType::CatmullRom,
                          ParameterizationType::chordLength);
  EXPECT_EQ(6947, int(spline.length()));
}

#endif /* TESTS_PARAMETRICSPLINETEST_H_ */
