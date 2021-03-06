#ifndef COORDS_COORDSCONVERTER_H_
#define COORDS_COORDSCONVERTER_H_

#include "../parametricSpline.h"
#include "cart.h"
#include "frenet.h"
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
  ParametricSpline* getSpline() const {
    return spline;
  }

  const MapWaypoints& getMapWaypoints() const {
    return map_waypoints;
  }

  double adapt_s_coord(double s_coord_from_simulator) const {
    const double overallLineLength_from_simulator = 6945.5540547387;
    return s_coord_from_simulator * spline->getLength()
        / overallLineLength_from_simulator;
  }

 private:
  Point getClockwisePerpendicular(Point v) const;

  const MapWaypoints& map_waypoints;
  ParametricSpline* spline;
};

CoordsConverter::CoordsConverter(const MapWaypoints& _map_waypoints)
    : map_waypoints(_map_waypoints) {
  spline = new ParametricSpline(map_waypoints.map_waypoints);
}

CoordsConverter::~CoordsConverter() {
  delete spline;
}

Frenet CoordsConverter::getFrenet(const Point& point) const {
  return spline->getFrenet(point);
}

Point CoordsConverter::getClockwisePerpendicular(Point v) const {
  return Point { v.y, -v.x };
}

Point CoordsConverter::getXY(const Frenet& point) const {
  double t = spline->toSplineParameter(point.s);
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
