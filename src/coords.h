#ifndef COORDS_H_
#define COORDS_H_

#include "main.h"
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

struct Point {
  double x;
  double y;

  static Point fromAngle(double angle_rad);
  double len() const;
  double scalarProd(const Point &point) const;
  double distanceTo(const Point &point) const;
  double getHeading() const;
  Point& operator=(const Point &point);
  Point operator+(const Point& other) const;
  Point operator-(const Point& other) const;
  Point operator*(double scalar) const;

  friend ostream& operator<<(ostream& os, const Point& point);
};

ostream& operator<<(ostream& os, const Point& point);

struct Frenet;
struct MapWaypoints;

Frenet getFrenet2(const Point& point, const MapWaypoints& map_waypoints);

#endif /* COORDS_H_ */
