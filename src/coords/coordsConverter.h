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

struct LineSegment {
  Point start;
  Point end;

  Point asVector() const;
  double len() const;
  Point getProjectedPoint(const Point& point) const;
  // TODO: make private
  double getFrenetS(const Point& point) const;
  double getFrenetD(const Point& point, const Point& v_outwards) const;
  Frenet getFrenet(const Point& point, const Point& v_outwards) const;
};

Point LineSegment::asVector() const {
  return end - start;
}

double LineSegment::len() const {
  return asVector().len();
}

Point LineSegment::getProjectedPoint(const Point& point) const {
  return start + asVector().asNormalized() * getFrenetS(point);
}

double LineSegment::getFrenetS(const Point& point) const {
  return asVector().asNormalized().scalarProd(point - start);
}

double LineSegment::getFrenetD(const Point& point,
                               const Point& v_outwards) const {
  Point d_vec = point - getProjectedPoint(point);
  return d_vec.len() * sgn(d_vec.scalarProd(v_outwards));
}

Frenet LineSegment::getFrenet(const Point& point,
                               const Point& v_outwards) const {

  return Frenet { getFrenetS(point), getFrenetD(point, v_outwards) };
}

class CoordsConverter {

 public:
  CoordsConverter(const MapWaypoints& map_waypoints);
  Frenet getFrenet(const Point& point) const;

 private:
  Frenet getFrenet(const LineSegment& lineSegment, const Point& point,
                   const Point& v_outwards, int waypointIndex) const;
  int getIndexOfClosestWaypoint(const Point& point) const;
  double getDistanceFromWaypointZeroToWaypoint(int waypointIndex) const;

  const MapWaypoints &map_waypoints;
};

CoordsConverter::CoordsConverter(const MapWaypoints& _map_waypoints)
    : map_waypoints(_map_waypoints) {
}

int CoordsConverter::getIndexOfClosestWaypoint(const Point& point) const {
  auto index_of_minimum = [](const vector<double>& v) {
    return std::distance(
        v.begin(),
        std::min_element(v.begin(), v.end()));
  };

  vector<double> distancesFromPoint2Waypoints = map2<Point, double>(
      map_waypoints.map_waypoints, [&point](const Point& p) {
        return point.distanceTo(p);
      });

  return index_of_minimum(distancesFromPoint2Waypoints);
}

int modulo(int n, int N) {
  return n >= 0 ? n % N : N - ((-n) % N);
}

bool isProjectionOfPointOntoLineWithinLineSegment(
    const Point& point, const LineSegment& lineSegment) {

  double s = lineSegment.getFrenetS(point);
  return 0 <= s && s <= lineSegment.len();
}

double CoordsConverter::getDistanceFromWaypointZeroToWaypoint(
    int waypointIndex) const {

  const vector<Point>& maps = map_waypoints.map_waypoints;
  double dist = 0;
  for (int i = 0; i < waypointIndex; i++) {
    dist += maps[i].distanceTo(maps[i + 1]);
  }
  return dist;
}

Frenet CoordsConverter::getFrenet(const LineSegment& lineSegment,
                                  const Point& point, const Point& v_outwards,
                                  int waypointIndex) const {
  return Frenet { getDistanceFromWaypointZeroToWaypoint(waypointIndex), 0 }
      + lineSegment.getFrenet(point, v_outwards);
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet CoordsConverter::getFrenet(const Point& point) const {
  int closestIndex = getIndexOfClosestWaypoint(point);
  int prevIndex = modulo(closestIndex - 1, map_waypoints.map_waypoints.size());
  int nextIndex = modulo(closestIndex + 1, map_waypoints.map_waypoints.size());

  Point closest = map_waypoints.map_waypoints[closestIndex];
  Point prev = map_waypoints.map_waypoints[prevIndex];
  Point next = map_waypoints.map_waypoints[nextIndex];

  auto isPointInSegmentPrev2Closest =
      [&]() {
        return isProjectionOfPointOntoLineWithinLineSegment(point, LineSegment {prev, closest});
      };

  auto getFrenetBasedOnSegmentPrev2Closest = [&]() {
    return getFrenet(LineSegment {prev, closest}, point,
        map_waypoints.map_outwards[prevIndex],
        prevIndex);
  };

  auto isPointInSegmentClosest2Next =
      [&]() {
        return isProjectionOfPointOntoLineWithinLineSegment(point, LineSegment {closest, next});
      };

  auto getFrenetBasedOnSegmentClosest2Next = [&]() {
    return getFrenet(LineSegment {closest, next}, point,
        map_waypoints.map_outwards[closestIndex],
        closestIndex);
  };

  auto getFrenetNearest2LineSegment =
      [&]() {
        return std::min(
            getFrenetBasedOnSegmentPrev2Closest(),
            getFrenetBasedOnSegmentClosest2Next(),
            [](const Frenet& frenet1, const Frenet& frenet2) {return fabs(frenet1.d) < fabs(frenet2.d);});
      };

  if (isPointInSegmentPrev2Closest()) {
    if (isPointInSegmentClosest2Next()) {
      return getFrenetNearest2LineSegment();
    } else {
      return getFrenetBasedOnSegmentPrev2Closest();
    }
  } else {
    return getFrenetBasedOnSegmentClosest2Next();
  }
}

#endif /* COORDS_COORDSCONVERTER_H_ */
