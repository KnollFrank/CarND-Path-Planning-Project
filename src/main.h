#ifndef MAIN_H_
#define MAIN_H_

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
#include "coords.h"
#include "car.h"

using namespace std;

enum Lane {
  LEFT = 0,
  MIDDLE = 1,
  RIGHT = 2
};

struct Path {
  vector<Point> points;
};

struct PreviousData {
  Path previous_path;
  Frenet end_path;
};

struct ReferencePoint {
  Point point;
  double yaw_rad;
  double vel_mph;
};

#endif /* MAIN_H_ */
