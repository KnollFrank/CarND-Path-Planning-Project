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
#include "lineSegment.h"

using namespace std;

class CoordsConverter {

 public:
  CoordsConverter(const MapWaypoints& map_waypoints);
  Frenet getFrenet(const Point& point) const;

 private:
  Frenet getFrenet(const LineSegment& lineSegment, const Point& point,
                   int waypointIndex) const;
  int getIndexOfClosestWaypoint(const Point& point) const;
  double getDistanceFromWaypointZeroToWaypoint(int waypointIndex) const;
  int adaptWaypointIndex(int waypointIndex) const;

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

bool isProjectionOfPointWithinLineSegment(const Point& point,
                                          const LineSegment& lineSegment) {

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
                                  const Point& point, int waypointIndex) const {
  return Frenet { getDistanceFromWaypointZeroToWaypoint(waypointIndex), 0 }
      + lineSegment.getFrenet(point, map_waypoints.map_outwards[waypointIndex]);
}

int CoordsConverter::adaptWaypointIndex(int waypointIndex) const {
  return modulo(waypointIndex, map_waypoints.map_waypoints.size());
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet CoordsConverter::getFrenet(const Point& point) const {
  const int closestIndex = getIndexOfClosestWaypoint(point);
  const int prevIndex = adaptWaypointIndex(closestIndex - 1);
  const int nextIndex = adaptWaypointIndex(closestIndex + 1);

  const LineSegment prev2closest = LineSegment {
      map_waypoints.map_waypoints[prevIndex],
      map_waypoints.map_waypoints[closestIndex] };
  const LineSegment closest2next = LineSegment {
      map_waypoints.map_waypoints[closestIndex],
      map_waypoints.map_waypoints[nextIndex] };

  auto getFrenetBasedOnSegmentPrev2Closest = [&]() {
    return getFrenet(prev2closest, point, prevIndex);
  };

  auto getFrenetBasedOnSegmentClosest2Next = [&]() {
    return getFrenet(closest2next, point, closestIndex);
  };

  auto getFrenetNearest2LineSegment =
      [&]() {
        return std::min(
            getFrenetBasedOnSegmentPrev2Closest(),
            getFrenetBasedOnSegmentClosest2Next(),
            [](const Frenet& frenet1, const Frenet& frenet2) {return fabs(frenet1.d) < fabs(frenet2.d);});
      };

  if (isProjectionOfPointWithinLineSegment(point, prev2closest)) {
    if (isProjectionOfPointWithinLineSegment(point, closest2next)) {
      return getFrenetNearest2LineSegment();
    } else {
      return getFrenetBasedOnSegmentPrev2Closest();
    }
  } else {
    return getFrenetBasedOnSegmentClosest2Next();
  }
}

#endif /* COORDS_COORDSCONVERTER_H_ */
