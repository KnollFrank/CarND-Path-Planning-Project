#ifndef COORDS_WAYPOINTS_H_
#define COORDS_WAYPOINTS_H_

#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <tuple>
#include <vector>

#include "cart.h"
#include "lineSegment.h"

using namespace std;

struct MapWaypoints {
  vector<Point> map_waypoints;
  vector<Point> map_outwards;
  vector<double> map_waypoints_s;

  static MapWaypoints load();
  double getDistanceFromWaypointZeroToWaypoint(int waypointIndex) const;
  LineSegment getLineSegment(int start, int end) const;
};

double MapWaypoints::getDistanceFromWaypointZeroToWaypoint(
    int waypointIndex) const {
  double dist = 0;
  for (int i = 0; i < waypointIndex; i++) {
    dist += map_waypoints[i].distanceTo(map_waypoints[i + 1]);
  }
  return dist;
}

LineSegment MapWaypoints::getLineSegment(int start, int end) const {
  return LineSegment { map_waypoints[start], map_waypoints[end] };
}

// Load up map values for waypoint's x,y,s and d normalized normal vectors
MapWaypoints MapWaypoints::load() {
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
    map_waypoints.map_outwards.push_back(Point { d_x, d_y });
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  return map_waypoints;
}

#endif /* COORDS_WAYPOINTS_H_ */
