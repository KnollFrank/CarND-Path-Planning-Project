#ifndef COORDS_LINESEGMENT_H_
#define COORDS_LINESEGMENT_H_

#include "cart.h"
#include "frenet.h"
#include "../mathfuns.h"

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

#endif /* COORDS_LINESEGMENT_H_ */
