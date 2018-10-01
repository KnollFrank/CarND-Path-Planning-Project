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
#include "funs.h"

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
  PolynomDescription getDerivative() const;
};

double PolynomDescription::operator()(double x) const {
  return poly.evaluate(x - start);
}

PolynomDescription PolynomDescription::getDerivative() const {
  vector<double> coeffs;
  for (int i = 1; i <= poly.degree(); i++) {
    coeffs.push_back(i * poly[i]);
  }
  polynomial<double> deriv(coeffs.begin(), coeffs.end());
  // TODO: introduce constructor
  PolynomDescription polyPrime;
  polyPrime.start = start;
  polyPrime.end = end;
  polyPrime.poly = deriv;
  return polyPrime;
}

struct PolynomDescription2D {
  PolynomDescription x;
  PolynomDescription y;
};

class ParametricSpline {

 public:
  ParametricSpline(const vector<Point>& points);

  Point operator()(double t) const;
  Point getTangent(double t) const;
  double length() const;
  double distanceTo(const Point& point);

 private:
  real_2d_array as_real_2d_array(const vector<Point> &points) const;
  alglib_impl::pspline2interpolant* asImplPtr() const;
  vector<PolynomDescription> createPolynomDescriptions(
      alglib_impl::spline1dinterpolant &spline) const;
  PolynomDescription getSquaredDistancePrimePoly(
      const Point& point, const PolynomDescription2D& poly);
  double getSquaredDistance(double t, const Point& point,
                            const PolynomDescription2D& poly);
  PolynomDescription getSquaredDistancePoly(const Point& point,
                                            const PolynomDescription2D& poly);
  double squaredDistancePrimeRoot(
      const PolynomDescription& squaredDistancePrime);
  vector<PolynomDescription> getXPolys() const;
  vector<PolynomDescription> getYPolys() const;
  vector<PolynomDescription2D> getPolys() const;

  pspline2interpolant spline;
};

vector<PolynomDescription> ParametricSpline::createPolynomDescriptions(
    alglib_impl::spline1dinterpolant &spline) const {

  ae_int_t n;
  real_2d_array tbl;
  xparams _xparams;
  spline1dunpack2(spline, n, tbl, _xparams);

  vector<PolynomDescription> polys;
  for (int i = 0; i < n - 1; i++) {
    PolynomDescription poly;
    poly.start = tbl(i, 0);
    poly.end = tbl(i, 1);
    poly.poly = { {tbl(i, 2), tbl(i, 3), tbl(i, 4), tbl(i, 5)}};
    polys.push_back(poly);

  }
  return polys;
}

alglib_impl::pspline2interpolant* ParametricSpline::asImplPtr() const {
  return const_cast<alglib_impl::pspline2interpolant*>(spline.c_ptr());
}

vector<PolynomDescription> ParametricSpline::getXPolys() const {
  return createPolynomDescriptions(asImplPtr()->x);
}

vector<PolynomDescription> ParametricSpline::getYPolys() const {
  return createPolynomDescriptions(asImplPtr()->y);
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

struct DistancePrimeFunctor {

  DistancePrimeFunctor(const PolynomDescription& _poly)
      : poly(_poly) {
  }

  std::pair<double, double> operator()(double x) {
    return std::make_pair(poly(x), poly.getDerivative()(x));
  }

 private:
  const PolynomDescription& poly;
};

double ParametricSpline::squaredDistancePrimeRoot(
    const PolynomDescription& squaredDistancePrime) {
  using namespace boost::math::tools;
  double min = squaredDistancePrime.start;
  double max = squaredDistancePrime.end;
  // guess is the root of the linear term of squaredDistancePrime
  // double guess = -squaredDistancePrime.poly[0] / squaredDistancePrime.poly[1];
  double guess = (min + max) / 2.0;
  const int digits = std::numeric_limits<double>::digits;
  int get_digits = static_cast<int>(digits * 0.6);
  const boost::uintmax_t maxit = 20;
  boost::uintmax_t it = maxit;
  DistancePrimeFunctor functor = DistancePrimeFunctor(squaredDistancePrime);
  return newton_raphson_iterate(functor, guess, min, max, get_digits/*, it*/);
}

PolynomDescription ParametricSpline::getSquaredDistancePoly(
    const Point& point, const PolynomDescription2D& poly) {
  PolynomDescription squaredDistance;
  squaredDistance.start = poly.x.start;
  squaredDistance.end = poly.x.end;
  squaredDistance.poly = pow(point.x - poly.x.poly, 2)
      + pow(point.y - poly.y.poly, 2);
  return squaredDistance;
}

PolynomDescription ParametricSpline::getSquaredDistancePrimePoly(
    const Point& point, const PolynomDescription2D& poly) {
  return getSquaredDistancePoly(point, poly).getDerivative();
}

double ParametricSpline::getSquaredDistance(double t, const Point& point,
                                            const PolynomDescription2D& poly) {
  return getSquaredDistancePoly(point, poly)(t);
}

vector<PolynomDescription2D> ParametricSpline::getPolys() const {
  vector<PolynomDescription2D> polys;
  vector<PolynomDescription> xpolys = getXPolys();
  vector<PolynomDescription> ypolys = getYPolys();
  for (int i = 0; i < xpolys.size(); i++) {
    PolynomDescription2D poly;
    poly.x = xpolys[i];
    poly.y = ypolys[i];
    polys.push_back(poly);
  }
  return polys;
}

double ParametricSpline::distanceTo(const Point& point) {
  auto get_min_element =
      [](const auto& v) {return std::min_element(v.begin(), v.end());};

  auto index_of_minimum = [&](const auto& v) {
    return std::distance(v.begin(), get_min_element(v));
  };

  vector<PolynomDescription2D> polys = getPolys();
  vector<double> distancesFromPoint2Polys =
      map2<PolynomDescription2D, double>(
          polys,
          [&](const PolynomDescription2D& poly) {
            PolynomDescription squaredDistancePrime = getSquaredDistancePrimePoly(point,
                poly);
            double root = squaredDistancePrimeRoot(squaredDistancePrime);
            vector<double> distances = map2<double, double>( {root, poly.x.start, poly.x.end}, [&](double x) {return sqrt(getSquaredDistance(x, point, poly));});
            return *get_min_element(distances);
          });

  int min_index = index_of_minimum(distancesFromPoint2Polys);
  return distancesFromPoint2Polys[min_index];
}

#endif /* PARAMETRICSPLINE_H_ */
