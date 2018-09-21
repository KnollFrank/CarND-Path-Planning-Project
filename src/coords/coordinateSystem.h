#ifndef COORDS_COORDINATESYSTEM_H_
#define COORDS_COORDINATESYSTEM_H_

#include "cart.h"
#include "frenet.h"
#include "../funs.h"
#include "waypoints.h"
#include "lineSegment.h"
#include "coordSys.h"

// TODO: merge with CoordianteSystemCart using templates
struct CoordinateSystem {
  Frenet origin;
  Frenet e1;
  Frenet e2;

  Frenet transform(double e1_coord, double e2_coord) const;
  Frenet transform(const Frenet& point) const;
};

Frenet CoordinateSystem::transform(double e1_coord, double e2_coord) const {
  return origin + e1 * e1_coord + e2 * e2_coord;
}

Frenet CoordinateSystem::transform(const Frenet& point) const {
  return transform(point.s, point.d);
}

#endif /* COORDS_COORDINATESYSTEM_H_ */
