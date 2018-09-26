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
  ParametricSpline(const vector<Point> &points, const SplineType st,
                   const ParameterizationType pt);

  Point operator()(double t) const;
  Point getTangent(double t) const;
  double length() const;

 private:
  real_2d_array as_real_2d_array(const vector<Point> &points) const;

  pspline2interpolant spline;
};

double ParametricSpline::length() const {
  return pspline2arclength(spline, 0, 1);
}

Point ParametricSpline::getTangent(double t) const {
  double x;
  double y;
  pspline2tangent(spline, t, x, y);
  return Point { x, y };
}

Point ParametricSpline::operator()(double t) const {
  double x;
  double y;
  pspline2calc(spline, t, x, y);
  return Point { x, y };
}

real_2d_array ParametricSpline::as_real_2d_array(const vector<Point> &points) const {
  real_2d_array xy;
  xy.setlength(points.size(), 2);
  for (int row = 0; row < points.size(); row++) {
    xy(row, 0) = points[row].x;
    xy(row, 1) = points[row].y;
  }
  return xy;
}

ParametricSpline::ParametricSpline(const vector<Point> &points, const SplineType st,
                                   const ParameterizationType pt) {
  real_2d_array xy = as_real_2d_array(points);
  pspline2buildperiodic(xy, points.size(), st, pt, spline);
}

#endif /* PARAMETRICSPLINE_H_ */
