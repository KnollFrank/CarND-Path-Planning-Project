#ifndef COORDS_LINESEGMENT_H_
#define COORDS_LINESEGMENT_H_

#include "cart.h"
#include "frenet.h"
#include "../mathfuns.h"

class LineSegment {
 public:
  const Point start;
  const Point end;

  double len() const;
  double getFrenetS(const Point& point) const;
  Frenet getFrenet(const Point& point, const Point& v_outwards) const;

 private:
  Point asVector() const;
  Point getProjectedPoint(const Point& point) const;
  double getFrenetD(const Point& point, const Point& v_outwards) const;
  Point getBasisVector() const;
};

Point LineSegment::asVector() const {
  return end - start;
}

double LineSegment::len() const {
  return asVector().len();
}

Point LineSegment::getBasisVector() const {
  return asVector().asNormalized();
}

Point LineSegment::getProjectedPoint(const Point& point) const {
  return start + getBasisVector() * getFrenetS(point);
}

double LineSegment::getFrenetS(const Point& point) const {
  return getBasisVector().scalarProd(point - start);
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
