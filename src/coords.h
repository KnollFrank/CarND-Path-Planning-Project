#ifndef COORDS_H_
#define COORDS_H_

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

struct Frenet {
  double s;
  double d;

  Frenet operator+(const Frenet& other) const;
  Frenet operator-(const Frenet& other) const;
  Frenet operator*(double scalar) const;

  friend ostream& operator<<(ostream& os, const Frenet& frenet);
};

struct MapWaypoints {
  vector<Point> map_waypoints;
  vector<double> map_waypoints_s;
};

bool operator==(const Frenet& lhs, const Frenet& rhs);
ostream& operator<<(ostream& os, const Frenet& frenet);
ostream& operator<<(ostream& os, const Point& point);
MapWaypoints read_map_waypoints();
Frenet getFrenet2(const Point& point, const MapWaypoints& map_waypoints);

#endif /* COORDS_H_ */
