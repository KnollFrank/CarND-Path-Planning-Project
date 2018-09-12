#ifndef COORDS_COORDSCONVERTER_H_
#define COORDS_COORDSCONVERTER_H_

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <functional>
#include <tuple>

#include "cart.h"
#include "frenet.h"
#include "../mathfuns.h"
#include "waypoints.h"

using namespace std;

class CoordsConverter {

 public:
  CoordsConverter(const MapWaypoints& map_waypoints);
  Frenet getFrenet(const Point& point) const;

 private:
  const MapWaypoints &map_waypoints;
};

CoordsConverter::CoordsConverter(const MapWaypoints& _map_waypoints)
    : map_waypoints(_map_waypoints) {
}

int ClosestWaypoint2(const Point &point, const MapWaypoints &map_waypoints) {
  auto isCloserToPoint = [&point](const Point& point1, const Point& point2) {
    double distance1 = point.distanceTo(point1);
    double distance2 = point.distanceTo(point2);
    return distance1 < distance2;
  };

  auto index_of_minimum =
      [&isCloserToPoint](const vector<Point> &points) {
        return std::distance(
            points.begin(),
            std::min_element(points.begin(), points.end(), isCloserToPoint));
      };

  return index_of_minimum(map_waypoints.map_waypoints);
}

int NextWaypoint2(const Point &point, double theta_rad,
                  const MapWaypoints &map_waypoints) {

  int closestWaypoint = ClosestWaypoint2(point, map_waypoints);
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
Frenet CoordsConverter::getFrenet(const Point& point) const {
  const vector<Point> &maps = map_waypoints.map_waypoints;
  int next_wp = NextWaypoint2(point, 0, map_waypoints);

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

#endif /* COORDS_COORDSCONVERTER_H_ */
