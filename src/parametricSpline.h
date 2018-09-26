#ifndef PARAMETRICSPLINE_H_
#define PARAMETRICSPLINE_H_

#include "alglib/ap.h"
#include "alglib/interpolation.h"
#include "coords/cart.h"

using namespace alglib;

enum SplineType {
  CatmullRom = 1,
  Cubic = 2
};

enum ParameterizationType {
  uniform = 0,
  chordLength = 1,
  centripetal = 2
};

class ParametricSpline {

 public:
  ParametricSpline(const real_2d_array &xy, const SplineType st,
                   const ParameterizationType pt);

  Point operator() (double t) const;
  Point getTangent(double t) const;

 // private:
  pspline2interpolant spline;
};

Point ParametricSpline::getTangent(double t) const {
  double x;
  double y;
  pspline2tangent(spline, t, x, y);
  return Point { x, y };
}

Point ParametricSpline::operator() (double t) const {
  double x;
  double y;
  pspline2calc(spline, t, x, y);
  return Point { x, y };
}

ParametricSpline::ParametricSpline(const real_2d_array &xy, const SplineType st,
                                   const ParameterizationType pt) {
  pspline2buildperiodic(xy, xy.rows(), st, pt, spline);
}

#endif /* PARAMETRICSPLINE_H_ */
