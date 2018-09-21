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
#include "../funs.h"
#include "waypoints.h"
#include "lineSegment.h"
#include "coordSys.h"
#include "coordinateSystem.h"
#include "coordinateSystemCart.h"

using namespace std;

class CoordsConverter {

 public:
  CoordsConverter(const MapWaypoints& map_waypoints);
  Frenet getFrenet(const Point& point) const;
  Point getXY(const Frenet& point) const;
  Point createCartVectorFromStart2End(const Frenet &start,
                                              const Frenet &end) const;
  Frenet createFrenetVectorFromStart2End(const Point &start,
                                                 const Point &end) const;

 private:
  int getIndexOfClosestWaypoint(const Point& point) const;
  int adaptWaypointIndex(int waypointIndex) const;
  LineSegment createLineSegment(const int startWaypointIndex,
                                const int endWaypointIndex) const;
  const CoordSys createCoordSys(const Point& point,
                                const int startWaypointIndex,
                                const int endWaypointIndex) const;
  int getStartIndex(const Frenet& point) const;
  Point getClockwisePerpendicular(Point v) const;
  CoordinateSystemCart createCoordinateSystem(const LineSegment& lineSegment) const;
  LineSegment getLineSegmentContaining(const Frenet& point) const;

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

int CoordsConverter::adaptWaypointIndex(int waypointIndex) const {
  return modulo(waypointIndex, map_waypoints.map_waypoints.size());
}

LineSegment CoordsConverter::createLineSegment(
    const int startWaypointIndex, const int endWaypointIndex) const {

  return LineSegment { map_waypoints.map_waypoints[startWaypointIndex],
      map_waypoints.map_waypoints[endWaypointIndex] };
}

const CoordSys CoordsConverter::createCoordSys(
    const Point& point, const int startWaypointIndex,
    const int endWaypointIndex) const {

  return CoordSys(map_waypoints, point,
                  createLineSegment(startWaypointIndex, endWaypointIndex),
                  startWaypointIndex);
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet CoordsConverter::getFrenet(const Point& point) const {
  const int closestIndex = getIndexOfClosestWaypoint(point);
  const CoordSys coordSysPrev2Closest = createCoordSys(
      point, adaptWaypointIndex(closestIndex - 1), closestIndex);
  const CoordSys coordSysClosest2Next = createCoordSys(
      point, closestIndex, adaptWaypointIndex(closestIndex + 1));

  auto getFrenetOfNearestLineSegment =
      [&]() {
        return std::min(
            coordSysPrev2Closest.getFrenet(),
            coordSysClosest2Next.getFrenet(),
            [](const Frenet& frenet1, const Frenet& frenet2) {return fabs(frenet1.d) < fabs(frenet2.d);});
      };

  if (coordSysPrev2Closest.isProjectionOfPointWithinLineSegment()) {
    if (coordSysClosest2Next.isProjectionOfPointWithinLineSegment()) {
      return getFrenetOfNearestLineSegment();
    } else {
      return coordSysPrev2Closest.getFrenet();
    }
  } else {
    return coordSysClosest2Next.getFrenet();
  }
}

int CoordsConverter::getStartIndex(const Frenet& point) const {
  int startIndex = -1;
  while (point.s > map_waypoints.map_waypoints_s[startIndex + 1]
      && (startIndex < (int) ((map_waypoints.map_waypoints_s.size() - 1)))) {
    startIndex++;
  }
  return startIndex;
}

Point CoordsConverter::getClockwisePerpendicular(Point v) const {
  return Point { v.y, -v.x };
}

CoordinateSystemCart CoordsConverter::createCoordinateSystem(
    const LineSegment& lineSegment) const {
  Point e1 = lineSegment.getBasisVector();
  // TODO: was ist, falls (dx, dy) in Richtung heading + pi() / 2 statt heading - pi() / 2 zeigen?
  return CoordinateSystemCart { lineSegment.start, e1, getClockwisePerpendicular(e1) };
}

LineSegment CoordsConverter::getLineSegmentContaining(
    const Frenet& point) const {
  int startIndex = getStartIndex(point);
  LineSegment lineSegment = map_waypoints.getLineSegment(
      startIndex, adaptWaypointIndex(startIndex + 1));
  return lineSegment;
}

Point CoordsConverter::getXY(const Frenet& point) const {
  LineSegment lineSegment = getLineSegmentContaining(point);
  CoordinateSystemCart coordinateSystem = createCoordinateSystem(lineSegment);
  return coordinateSystem.transform(
      point.s - map_waypoints.map_waypoints_s[getStartIndex(point)], point.d);
}

Point CoordsConverter::createCartVectorFromStart2End(
    const Frenet &start, const Frenet &end) const {
  return getXY(end) - getXY(start);
}

// TODO: replace " &" with "& " in all files
Frenet CoordsConverter::createFrenetVectorFromStart2End(
    const Point &start, const Point &end) const {
  return getFrenet(end) - getFrenet(start);
}

#endif /* COORDS_COORDSCONVERTER_H_ */
