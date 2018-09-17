#ifndef COORDS_COORDINATESYSTEM_H_
#define COORDS_COORDINATESYSTEM_H_

#include "cart.h"
#include "frenet.h"
#include "../mathfuns.h"
#include "waypoints.h"
#include "lineSegment.h"
#include "coordSys.h"

struct CoordinateSystem {
  Point e1;
  Point e2;

  Point transform(double e1_coord, double e2_coord) const;
};

Point CoordinateSystem::transform(double e1_coord, double e2_coord) const {
  return e1 * e1_coord + e2 * e2_coord;
}

#endif /* COORDS_COORDINATESYSTEM_H_ */
