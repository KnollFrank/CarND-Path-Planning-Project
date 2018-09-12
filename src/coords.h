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
#include "cart.h"
#include "frenet.h"

using namespace std;

struct MapWaypoints {
  vector<Point> map_waypoints;
  vector<double> map_waypoints_s;
};

// Load up map values for waypoint's x,y,s and d normalized normal vectors
MapWaypoints read_map_waypoints() {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  MapWaypoints map_waypoints;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints.map_waypoints.push_back(Point { x, y });
    map_waypoints.map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  return map_waypoints;
}

bool operator==(const Frenet& lhs, const Frenet& rhs);
ostream& operator<<(ostream& os, const Frenet& frenet);
ostream& operator<<(ostream& os, const Point& point);
MapWaypoints read_map_waypoints();

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

int ClosestWaypoint(const Point &point, const MapWaypoints &map_waypoints) {

  double closestLen = 100000;  //large number
  int closestWaypoint = 0;

  for (int i = 0; i < map_waypoints.map_waypoints.size(); i++) {
    double dist = point.distanceTo(map_waypoints.map_waypoints[i]);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(const Point &point, double theta_rad,
                 const MapWaypoints &map_waypoints) {

  int closestWaypoint = ClosestWaypoint(point, map_waypoints);
  Point map = map_waypoints.map_waypoints[closestWaypoint];
  double heading = (map - point).getHeading();
  double angle = fabs(theta_rad - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == map_waypoints.map_waypoints.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet getFrenet(const Point &point, double theta_rad,
                 const MapWaypoints &map_waypoints) {
  const vector<Point> &maps = map_waypoints.map_waypoints;
  int next_wp = NextWaypoint(point, theta_rad, map_waypoints);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps.size() - 1;
  }

  const Point n = maps[next_wp] - maps[prev_wp];
  const Point x = point - maps[prev_wp];

  // find the projection of x onto n
  // TODO: warum nicht /n.len() ?
  double proj_norm = x.scalarProd(n) / n.scalarProd(n);
  const Point proj = n * proj_norm;
  double frenet_d = x.distanceTo(proj);

  //see if d value is positive or negative by comparing it to a center point

  const Point center = Point { 1000, 2000 } - maps[prev_wp];
  double centerToPos = center.distanceTo(x);
  double centerToRef = center.distanceTo(proj);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += maps[i].distanceTo(maps[i + 1]);
  }

  frenet_s += proj.len();

  return Frenet { frenet_s, frenet_d };
}

Point createCartVectorConnectingStartAndEnd(const Frenet &start,
                                            const Frenet &end,
                                            const MapWaypoints &map_waypoints) {
  return getXY(end, map_waypoints) - getXY(start, map_waypoints);
}

Frenet createFrenetVectorConnectingStartAndEnd(
    const Point &start, const Point &end, const MapWaypoints &map_waypoints) {

  return getFrenet(end, 0, map_waypoints) - getFrenet(start, 0, map_waypoints);
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet getFrenet2(const Point& point, const MapWaypoints& map_waypoints) {
  const vector<Point> &maps = map_waypoints.map_waypoints;
  int next_wp = NextWaypoint(point, 0, map_waypoints);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps.size() - 1;
  }

  const Point n = maps[next_wp] - maps[prev_wp];
  const Point x = point - maps[prev_wp];

  // find the projection of x onto n
  // TODO: warum nicht /n.len() ?
  double proj_norm = x.scalarProd(n) / n.scalarProd(n);
  const Point proj = n * proj_norm;
  double frenet_d = x.distanceTo(proj);

  //see if d value is positive or negative by comparing it to a center point

  const Point center = Point { 1000, 2000 } - maps[prev_wp];
  double centerToPos = center.distanceTo(x);
  double centerToRef = center.distanceTo(proj);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += maps[i].distanceTo(maps[i + 1]);
  }

  frenet_s += proj.len();

  return Frenet { frenet_s, frenet_d };
}

#endif /* COORDS_H_ */
