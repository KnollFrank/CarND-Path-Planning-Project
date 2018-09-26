#ifndef COORDS_COORDSCONVERTER_H_
#define COORDS_COORDSCONVERTER_H_

#include "/usr/include/c++/5/algorithm"
#include "/usr/include/c++/5/cmath"
#include "/usr/include/c++/5/iterator"
#include "/usr/include/c++/5/tr1/unordered_map"
#include "/usr/include/c++/5/vector"
#include "../alglib/ap.h"
#include "../funs.h"
#include "../parametricSpline.h"
#include "cart.h"
#include "coordinateSystemCart.h"
#include "coordSys.h"
#include "frenet.h"
#include "lineSegment.h"
#include "waypoints.h"

using namespace std;

class CoordsConverter {

 public:
  CoordsConverter(const MapWaypoints& map_waypoints);
  ~CoordsConverter();

  Frenet getFrenet(const Point& point) const;
  Point getXY(const Frenet& point) const;
  Point createCartVectorFromStart2End(const Frenet& start,
                                      const Frenet& end) const;
  Frenet createFrenetVectorFromStart2End(const Point& start,
                                         const Point& end) const;

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
  CoordinateSystemCart createCoordinateSystem(
      const LineSegment& lineSegment) const;
  LineSegment getLineSegmentContaining(const Frenet& point) const;
  void fillXYFromWaypoints(real_2d_array& xy);

  const MapWaypoints& map_waypoints;
  ParametricSpline* spline;
  double splineLength;
};

void CoordsConverter::fillXYFromWaypoints(real_2d_array& xy) {
  xy.setlength(map_waypoints.map_waypoints.size(), 2);
  for (int row = 0; row < map_waypoints.map_waypoints.size(); row++) {
    xy(row, 0) = map_waypoints.map_waypoints[row].x;
    xy(row, 1) = map_waypoints.map_waypoints[row].y;
  }
}

CoordsConverter::CoordsConverter(const MapWaypoints& _map_waypoints)
    : map_waypoints(_map_waypoints) {
  real_2d_array xy;
  fillXYFromWaypoints(xy);
  spline = new ParametricSpline(xy, SplineType::CatmullRom,
                                ParameterizationType::uniform);

  splineLength = spline->length();
}

CoordsConverter::~CoordsConverter() {
  delete spline;
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
  return CoordinateSystemCart { lineSegment.start, e1,
      getClockwisePerpendicular(e1) };
}

LineSegment CoordsConverter::getLineSegmentContaining(
    const Frenet& point) const {
  int startIndex = getStartIndex(point);
  LineSegment lineSegment = map_waypoints.getLineSegment(
      startIndex, adaptWaypointIndex(startIndex + 1));
  return lineSegment;
}

Point CoordsConverter::getXY(const Frenet& point) const {
  double t = (point.s + 34.128) / splineLength;
  Point n = getClockwisePerpendicular(spline->getTangent(t));
  return (*spline)(t) + n * point.d;
}

Point CoordsConverter::createCartVectorFromStart2End(const Frenet& start,
                                                     const Frenet& end) const {
  return getXY(end) - getXY(start);
}

Frenet CoordsConverter::createFrenetVectorFromStart2End(
    const Point& start, const Point& end) const {
  return getFrenet(end) - getFrenet(start);
}

#endif /* COORDS_COORDSCONVERTER_H_ */
