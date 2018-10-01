#ifndef PARAMETRICSPLINE_H_
#define PARAMETRICSPLINE_H_

#include "alglib/ap.h"
#include "alglib/interpolation.h"
#include "coords/cart.h"

#include <boost/math/tools/polynomial.hpp>
#include <boost/math/tools/roots.hpp>

#include <boost/math/special_functions/next.hpp> // For float_distance.
#include <tuple> // for std::tuple and std::make_tuple.
#include <boost/math/special_functions/cbrt.hpp> // For boost::math::cbrt.

#include <iostream>
#include <iomanip>
#include <limits>

using namespace alglib;
using namespace boost::math::tools;

enum SplineType {
  CatmullRom = 1,
  Cubic = 2
};

enum ParameterizationType {
  uniform = 0,
  chordLength = 1,
  centripetal = 2
};

void spline1dunpack2(alglib_impl::spline1dinterpolant &c, ae_int_t &n,
                     real_2d_array &tbl, const xparams _xparams) {
  jmp_buf _break_jump;
  alglib_impl::ae_state _alglib_env_state;
  alglib_impl::ae_state_init(&_alglib_env_state);
  if (setjmp(_break_jump)) {
#if !defined(AE_NO_EXCEPTIONS)
    _ALGLIB_CPP_EXCEPTION(_alglib_env_state.error_msg);
#else
    _ALGLIB_SET_ERROR_FLAG(_alglib_env_state.error_msg);
    return;
#endif
  }
  ae_state_set_break_jump(&_alglib_env_state, &_break_jump);
  if (_xparams.flags != 0x0)
    ae_state_set_flags(&_alglib_env_state, _xparams.flags);
  alglib_impl::spline1dunpack(&c, &n,
                              const_cast<alglib_impl::ae_matrix*>(tbl.c_ptr()),
                              &_alglib_env_state);
  alglib_impl::ae_state_clear(&_alglib_env_state);
  return;
}

struct PolynomDescription {

  double start;
  double end;
  polynomial<double> poly;

  double operator()(double x) const;
};

double PolynomDescription::operator()(double x) const {
  return poly.evaluate(x - start);
}

class ParametricSpline {

 public:
  ParametricSpline(const vector<Point>& points);

  Point operator()(double t) const;
  Point getTangent(double t) const;
  double length() const;
  double distanceTo(const Point& point);
  static PolynomDescription derivation(const PolynomDescription& poly);

 private:
  real_2d_array as_real_2d_array(const vector<Point> &points) const;
  alglib_impl::pspline2interpolant* asImplPtr() const;
  PolynomDescription createPolynomDescription(
      alglib_impl::spline1dinterpolant &spline) const;
  PolynomDescription getSquaredDistancePrimePoly(const Point& point,
                                                 const PolynomDescription& x,
                                                 const PolynomDescription& y);
  double getSquaredDistance(double t, const Point& point,
                            const PolynomDescription& x,
                            const PolynomDescription& y);
  PolynomDescription getSquaredDistancePoly(const Point& point,
                                            const PolynomDescription& x,
                                            const PolynomDescription& y);
  double squaredDistancePrimeRoot(
      const PolynomDescription& squaredDistancePrime, double length);
  PolynomDescription getXPoly() const;
  PolynomDescription getYPoly() const;

  pspline2interpolant spline;
};

PolynomDescription ParametricSpline::createPolynomDescription(
    alglib_impl::spline1dinterpolant &spline) const {

  ae_int_t n;
  real_2d_array tbl;
  xparams _xparams;
  spline1dunpack2(spline, n, tbl, _xparams);
  PolynomDescription polyDescr;
  polyDescr.start = tbl(0, 0);
  polyDescr.end = tbl(0, 1);
  polyDescr.poly = { {tbl(0, 2), tbl(0, 3), tbl(0, 4), tbl(0, 5)}};
  return polyDescr;
}

alglib_impl::pspline2interpolant* ParametricSpline::asImplPtr() const {
  return const_cast<alglib_impl::pspline2interpolant*>(spline.c_ptr());
}

PolynomDescription ParametricSpline::getXPoly() const {
  return createPolynomDescription(asImplPtr()->x);
}

PolynomDescription ParametricSpline::getYPoly() const {
  return createPolynomDescription(asImplPtr()->y);
}

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

PolynomDescription ParametricSpline::derivation(
    const PolynomDescription& poly) {
  vector<double> coeffs;
  for (int i = 1; i <= poly.poly.degree(); i++) {
    coeffs.push_back(i * poly.poly[i]);
  }
  polynomial<double> deriv(coeffs.begin(), coeffs.end());
  // TODO: introduce constructor
  PolynomDescription polyPrime;
  polyPrime.start = poly.start;
  polyPrime.end = poly.end;
  polyPrime.poly = deriv;
  return polyPrime;
}

struct DistancePrimeFunctor {

  DistancePrimeFunctor(const PolynomDescription& _poly)
      : poly(_poly) {
  }

  std::pair<double, double> operator()(double x) {
    return std::make_pair(poly(x), ParametricSpline::derivation(poly)(x));
  }

 private:
  const PolynomDescription& poly;
};

double ParametricSpline::squaredDistancePrimeRoot(
    const PolynomDescription& squaredDistancePrime, double length) {
  using namespace boost::math::tools;
  // double guess = -squaredDistancePrime[0] / squaredDistancePrime[1];
//	double min = 120.0 / length;
//	double max = 150.0 / length;
//	double guess = 124.0 / length;
  double min = 0;
  double max = 30.0 / length;
  double guess = 25.0 / length;
  const int digits = std::numeric_limits<double>::digits;
  int get_digits = static_cast<int>(digits * 0.6);
  const boost::uintmax_t maxit = 20;
  boost::uintmax_t it = maxit;
  DistancePrimeFunctor functor = DistancePrimeFunctor(squaredDistancePrime);
  double result = newton_raphson_iterate(functor, guess, min, max,
                                         get_digits/*, it*/);
  pair<double, double> tmp = functor(result);
  return result;
}

PolynomDescription ParametricSpline::getSquaredDistancePoly(
    const Point& point, const PolynomDescription& x,
    const PolynomDescription& y) {
  PolynomDescription squaredDistance;
  squaredDistance.start = x.start;
  squaredDistance.end = x.end;
  squaredDistance.poly = pow(point.x - x.poly, 2) + pow(point.y - y.poly, 2);
  return squaredDistance;
}

PolynomDescription ParametricSpline::getSquaredDistancePrimePoly(
    const Point& point, const PolynomDescription& x,
    const PolynomDescription& y) {
  return derivation(getSquaredDistancePoly(point, x, y));
}

// TODO: const SplineDescription& x und const SplineDescription& y zu struct oder class ParametricSplineDescription zusammenfassen.
double ParametricSpline::getSquaredDistance(double t, const Point& point,
                                            const PolynomDescription& x,
                                            const PolynomDescription& y) {
  return getSquaredDistancePoly(point, x, y)(t);
}

double ParametricSpline::distanceTo(const Point& point) {
  PolynomDescription x = getXPoly();
  PolynomDescription y = getYPoly();

  PolynomDescription squaredDistancePrime = getSquaredDistancePrimePoly(point,
                                                                        x, y);
  double root = squaredDistancePrimeRoot(squaredDistancePrime, length());
  if (x.start < root && root < x.end) {
    cout << "Juhu;" << endl;
  }
  double dist1 = sqrt(getSquaredDistance(root, point, x, y));
  double dist2 = sqrt(getSquaredDistance(x.start, point, x, y));
  double dist3 = sqrt(getSquaredDistance(x.end, point, x, y));
  return dist1;
}

#endif /* PARAMETRICSPLINE_H_ */
