#ifndef COORDS_COORDINATESYSTEM_H_
#define COORDS_COORDINATESYSTEM_H_

#include "cart.h"
#include "frenet.h"
#include "../mathfuns.h"
#include "waypoints.h"
#include "lineSegment.h"
#include "coordSys.h"

struct CoordinateSystem {
  Point origin;
  Point e1;
  Point e2;

  Point transform(double e1_coord, double e2_coord) const;
  Point transform(const Point& point) const;
};

Point CoordinateSystem::transform(double e1_coord, double e2_coord) const {
  return origin + e1 * e1_coord + e2 * e2_coord;
}

Point CoordinateSystem::transform(const Point& point) const {
  return transform(point.x, point.y);
}

#endif /* COORDS_COORDINATESYSTEM_H_ */
