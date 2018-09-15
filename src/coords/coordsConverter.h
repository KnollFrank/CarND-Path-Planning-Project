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
  Frenet getFrenet(const Point& ontoA, const Point& B, const Point& v_outwards,
                   int index) const;

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

Frenet getFrenet2(const Point& ontoA, const Point& B, const Point& v_outwards) {
  // TODO: DRY with isProjectionOfBOntoAWithinA
  Point A_norm = ontoA.asNormalized();
  double s = A_norm.scalarProd(B);
  const Point B_proj = A_norm * s;
  Point B_perpendicular = B - B_proj;
  double d = B_perpendicular.len()
      * sgn(B_perpendicular.scalarProd(v_outwards));
  return Frenet { s, d };
}

Frenet CoordsConverter::getFrenet(const Point& ontoA, const Point& B,
                                  const Point& v_outwards, int index) const {
  const vector<Point> &maps = map_waypoints.map_waypoints;
  double dist = 0;
  for (int i = 0; i < index; i++) {
    dist += maps[i].distanceTo(maps[i + 1]);
  }

  return Frenet { dist, 0 } + getFrenet2(ontoA, B, v_outwards);
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet CoordsConverter::getFrenet(const Point& point) const {
  int closestIndex = getIndexOfClosestWaypoint(point, map_waypoints);
  int prevIndex = modulo(closestIndex - 1, map_waypoints.map_waypoints.size());
  int nextIndex = modulo(closestIndex + 1, map_waypoints.map_waypoints.size());
  Point closest = map_waypoints.map_waypoints[closestIndex];
  Point prev = map_waypoints.map_waypoints[prevIndex];
  Point next = map_waypoints.map_waypoints[nextIndex];
  auto getF1 = [&]() {
    return getFrenet(closest - prev, point - prev,
        map_waypoints.map_outwards[prevIndex],
        prevIndex);
  };

  auto getF2 = [&]() {
    return getFrenet(next - closest, point - closest,
        map_waypoints.map_outwards[closestIndex],
        closestIndex);
  };

  bool pointInSegment1 = isProjectionOfBOntoAWithinA(point - prev,
                                                     closest - prev);
  if (pointInSegment1) {
    bool pointInSegment2 = isProjectionOfBOntoAWithinA(point - closest,
                                                       next - prev);
    if (pointInSegment2) {
      return std::min(
          getF1(),
          getF2(),
          [](const Frenet& f1, const Frenet& f2) {return fabs(f1.d) < fabs(f2.d);});
    } else {
      return getF1();
    }
  } else {
    return getF2();
  }
}

#endif /* COORDS_COORDSCONVERTER_H_ */
