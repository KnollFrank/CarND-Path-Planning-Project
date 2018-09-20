#ifndef COORDS_COORDINATESYSTEMCART_H_
#define COORDS_COORDINATESYSTEMCART_H_

#include "cart.h"
#include "../mathfuns.h"
#include "waypoints.h"
#include "lineSegment.h"
#include "coordSys.h"

struct CoordinateSystemCart {
  Point origin;
  Point e1;
  Point e2;

  Point transform(double e1_coord, double e2_coord) const;
  Point transform(const Point& point) const;
};

Point CoordinateSystemCart::transform(double e1_coord, double e2_coord) const {
  return origin + e1 * e1_coord + e2 * e2_coord;
}

Point CoordinateSystemCart::transform(const Point& point) const {
  return transform(point.x, point.y);
}

#endif /* COORDS_COORDINATESYSTEMCART_H_ */
