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

vector<double> get_distances_of_point2points(const Point &point,
                                             vector<Point> points) {
  return map2<Point, double>(points, [&point](const Point& p) {
    return point.distanceTo(p);
  });
}

int getIndexOfClosestWaypoint(const Point &point,
                              const MapWaypoints &map_waypoints) {
  auto index_of_minimum = [](const vector<double> &v) {
    return std::distance(
        v.begin(),
        std::min_element(v.begin(), v.end()));
  };

  return index_of_minimum(
      get_distances_of_point2points(point, map_waypoints.map_waypoints));
}

int modulo(int n, int N) {
  return n >= 0 ? n % N : N - ((-n) % N);
}

bool isProjectionOfBOntoAWithinA(const Point& B, const Point& A) {
  double s = A.scalarProd(B) / A.len();
  return 0 <= s && s <= A.len();
}

int sgn(double n) {
  return n >= 0 ? +1 : -1;
}

Frenet getFrenet(const Point& ontoA, const Point& B, const Point& v_outwards) {
  // TODO: DRY with isProjectionOfBOntoAWithinA
  double s = ontoA.scalarProd(B) / ontoA.len();
  // TODO: neue Methode A.norm(), d ie A normalisiert.
  // const Point A_norm = A.norm();
  // double s = A_norm.scalarProd(B);
  // const Point B_proj = A_norm * s;
  const Point B_proj = ontoA * (1.0 / ontoA.len()) * s;
  double d = (B - B_proj).len() * sgn((B - B_proj).scalarProd(v_outwards));
  return Frenet { s, d };
}

Frenet getFrenet_hat(const Point& ontoA, const Point& B,
                     const Point& v_outwards, int index,
                     const MapWaypoints &map_waypoints) {
  const vector<Point> &maps = map_waypoints.map_waypoints;
  double dist = 0;
  for (int i = 0; i < index; i++) {
    dist += maps[i].distanceTo(maps[i + 1]);
  }

  return Frenet { dist, 0 } + getFrenet(ontoA, B, v_outwards);
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet CoordsConverter::getFrenet(const Point& point) const {
  int closestIndex = getIndexOfClosestWaypoint(point, map_waypoints);
  int prevIndex = modulo(closestIndex - 1, map_waypoints.map_waypoints.size());
  int nextIndex = modulo(closestIndex + 1, map_waypoints.map_waypoints.size());
  Point closest = map_waypoints.map_waypoints[closestIndex];
  Point prev = map_waypoints.map_waypoints[prevIndex];
  Point next = map_waypoints.map_waypoints[nextIndex];
  bool pointInSegment1 = isProjectionOfBOntoAWithinA(point - prev,
                                                     closest - prev);
  auto getF1 = [&]() {
    return getFrenet_hat(closest - prev, point - prev,
        map_waypoints.map_outwards[prevIndex],
        prevIndex, map_waypoints);
  };

  auto getF2 = [&]() {
    return getFrenet_hat(next - closest, point - closest,
        map_waypoints.map_outwards[closestIndex],
        closestIndex, map_waypoints);
  };

  if (pointInSegment1) {
    bool pointInSegment2 = isProjectionOfBOntoAWithinA(point - closest,
                                                       next - prev);
    if (pointInSegment2) {
      Frenet f1 = getF1();
      Frenet f2 = getF2();
      return fabs(f1.d) < fabs(f2.d) ? f1 : f2;
    } else {
      return getF1();
    }
  } else {
    return getF2();
  }
}

#endif /* COORDS_COORDSCONVERTER_H_ */
