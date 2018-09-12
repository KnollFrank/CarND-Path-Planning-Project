#ifndef WAYPOINTS_H_
#define WAYPOINTS_H_

#include "coords.h"
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

using namespace std;

struct Point;

struct MapWaypoints {
  vector<Point> map_waypoints;
  vector<double> map_waypoints_s;
};

MapWaypoints read_map_waypoints();

#endif /* WAYPOINTS_H_ */
