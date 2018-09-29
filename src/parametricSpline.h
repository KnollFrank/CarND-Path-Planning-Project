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
  ParametricSpline(const vector<Point>& points);

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

real_2d_array ParametricSpline::as_real_2d_array(
    const vector<Point> &points) const {

  real_2d_array xy;
  xy.setlength(points.size(), 2);
  for (int row = 0; row < points.size(); row++) {
    xy(row, 0) = points[row].x;
    xy(row, 1) = points[row].y;
  }
  return xy;
}

ParametricSpline::ParametricSpline(const vector<Point>& points) {
  real_2d_array xy = as_real_2d_array(points);
  pspline2buildperiodic(xy, points.size(), SplineType::CatmullRom,
                        ParameterizationType::chordLength, spline);
}

vector<double> polySquared(const vector<double>& a) {
  vector<double> d(7);
  d[0] = a[0] * a[0];
  d[1] = 2 * a[1] * a[0];
  d[2] = 2 * a[2] * a[0] + a[1] * a[1];
  d[3] = 2 * a[3] * a[0] + 2 * a[2] * a[1];
  d[4] = 2 * a[3] * a[1] + a[2] * a[2];
  d[5] = 2 * a[3] * a[2];
  d[6] = a[3] * a[3];
  return d;
}

vector<double> getDistancePrime(const Point& point, const vector<double>& a,
                                const vector<double>& b) {
  vector<double> d = polySquared(a);
  vector<double> e = polySquared(b);
  vector<double> distancePrime(6);
  distancePrime[0] = -2 * (point.x * a[1] + d[1] + point.y * b[1] + e[1]);
  distancePrime[1] = 2
      * (-2 * point.x * a[2] + d[2] - 2 * point.y * b[2] + e[2]);
  distancePrime[2] = 3
      * (-2 * point.x * a[3] + d[3] - 2 * point.y * b[3] + e[3]);
  distancePrime[3] = 4 * (d[4] + e[4]);
  distancePrime[4] = 5 * (d[5] + e[5]);
  distancePrime[5] = 6 * (d[6] + e[6]);
  return distancePrime;
}

double distance(const Point& point, const ParametricSpline& spline) {
  vector<double> a(4);
  vector<double> b(4);
  vector<double> distancePrime = getDistancePrime(point, a, b);
  return 4711;
}

#endif /* PARAMETRICSPLINE_H_ */
