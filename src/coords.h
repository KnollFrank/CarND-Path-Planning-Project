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
#include "mathfuns.h"

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

// Transform from Frenet s,d coordinates to Cartesian x,y
Point getXY(const Frenet &pos, const MapWaypoints &map_waypoints) {
  const vector<double> &maps_s = map_waypoints.map_waypoints_s;
  const vector<Point> &maps = map_waypoints.map_waypoints;

  int prev_wp = -1;

  while (pos.s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps.size();

  double heading = (maps[wp2] - maps[prev_wp]).getHeading();
  // the x,y,s along the segment
  double seg_s = pos.s - maps_s[prev_wp];

  Point seg = maps[prev_wp] + Point::fromAngle(heading) * seg_s;
  double perp_heading = heading - pi() / 2;

  return seg + Point::fromAngle(perp_heading) * pos.d;
}
#endif /* COORDS_H_ */
