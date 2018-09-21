#ifndef COORDS_COORDSYS_H_
#define COORDS_COORDSYS_H_

#include "cart.h"
#include "frenet.h"
#include "../funs.h"
#include "waypoints.h"
#include "lineSegment.h"

class CoordSys {
 public:
  CoordSys(const MapWaypoints& map_waypoints, const Point& point,
           const LineSegment& lineSegment, int waypointIndex);

  Frenet getFrenet() const;
  bool isProjectionOfPointWithinLineSegment() const;

 private:
  double getDistanceFromWaypointZeroToWaypoint() const;

  const MapWaypoints &map_waypoints;
  const Point& point;
  const LineSegment lineSegment;
  int waypointIndex;
};

CoordSys::CoordSys(const MapWaypoints& _map_waypoints, const Point& _point,
                   const LineSegment& _lineSegment, int _waypointIndex)
    : map_waypoints(_map_waypoints),
      point(_point),
      lineSegment(_lineSegment),
      waypointIndex(_waypointIndex) {
}

Frenet CoordSys::getFrenet() const {
  return Frenet { map_waypoints.getDistanceFromWaypointZeroToWaypoint(
      waypointIndex), 0 }
      + lineSegment.getFrenet(point, map_waypoints.map_outwards[waypointIndex]);
}

bool CoordSys::isProjectionOfPointWithinLineSegment() const {
  double s = lineSegment.getFrenetS(point);
  return 0 <= s && s <= lineSegment.len();
}

#endif /* COORDS_COORDSYS_H_ */
