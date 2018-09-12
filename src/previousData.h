#ifndef PREVIOUSDATA_H_
#define PREVIOUSDATA_H_

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <tuple>
#include "spline.h"
#include "car.h"
#include "mathfuns.h"
#include "lane.h"
#include "path.h"

using namespace std;

// for convenience
using json = nlohmann::json;

struct PreviousData {
  Path previous_path;
  Frenet end_path;
};

PreviousData createPreviousData(
    const nlohmann::basic_json<std::map, std::vector,
        std::__cxx11::basic_string<char, std::char_traits<char>,
            std::allocator<char> >, bool, long, unsigned long, double,
        std::allocator, nlohmann::adl_serializer> &j) {
  PreviousData previousData;
  // Previous path data given to the Planner
  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector<double> previous_path_y = j[1]["previous_path_y"];
  for (int i = 0; i < previous_path_x.size(); i++) {
    previousData.previous_path.points.push_back(Point { previous_path_x[i],
        previous_path_y[i] });
  }

  // Previous path's end s and d values
  previousData.end_path = Frenet { j[1]["end_path_s"], j[1]["end_path_d"] };
  return previousData;
}

#endif /* PREVIOUSDATA_H_ */
