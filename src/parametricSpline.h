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

struct Polynom {

  double start;
  double end;
  polynomial<double> poly;

  double operator()(double x) const;
  Polynom getDerivative() const;
};

double Polynom::operator()(double x) const {
  return poly.evaluate(x - start);
}

Polynom Polynom::getDerivative() const {
  vector<double> coeffs;
  for (int i = 1; i <= poly.degree(); i++) {
    coeffs.push_back(i * poly[i]);
  }
  polynomial<double> deriv(coeffs.begin(), coeffs.end());
  // TODO: introduce constructor
  Polynom polyPrime;
  polyPrime.start = start;
  polyPrime.end = end;
  polyPrime.poly = deriv;
  return polyPrime;
}

struct Polynom2D {
  Polynom x;
  Polynom y;
};

class ParametricSpline {

 public:
  ParametricSpline(const vector<Point>& points);

  Point operator()(double t) const;
  Point getTangent(double t) const;
  double getLength() const;
  double distanceTo(const Point& point) const;

 private:
  real_2d_array as_real_2d_array(const vector<Point> &points) const;
  alglib_impl::pspline2interpolant* asImplPtr() const;
  vector<Polynom> createPolynoms(
      alglib_impl::spline1dinterpolant &spline) const;
  Polynom getSquaredDistance(const Point& point, const Polynom2D& poly) const;
  double getRootOf(const Polynom& poly) const;
  vector<Polynom> getXPolys() const;
  vector<Polynom> getYPolys() const;
  vector<Polynom2D> getPolys() const;
  double getDistanceOfPoly2Point(const Polynom2D& poly,
                                 const Point& point) const;
  vector<double> getDistancesOfPolys2Point(const vector<Polynom2D>&,
                                           const Point& point) const;

  pspline2interpolant spline;
};

vector<Polynom> ParametricSpline::createPolynoms(
    alglib_impl::spline1dinterpolant &spline) const {

  ae_int_t n;
  real_2d_array tbl;
  xparams _xparams;
  spline1dunpack2(spline, n, tbl, _xparams);

  vector<Polynom> polys;
  for (int i = 0; i < n - 1; i++) {
    Polynom poly;
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

vector<Polynom> ParametricSpline::getXPolys() const {
  return createPolynoms(asImplPtr()->x);
}

vector<Polynom> ParametricSpline::getYPolys() const {
  return createPolynoms(asImplPtr()->y);
}

double ParametricSpline::getLength() const {
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

  DistancePrimeFunctor(const Polynom& _poly)
      : poly(_poly) {
    polyDerivative = poly.getDerivative();
  }

  std::pair<double, double> operator()(double x) {
    return std::make_pair(poly(x), polyDerivative(x));
  }

 private:
  const Polynom& poly;
  Polynom polyDerivative;
};

double ParametricSpline::getRootOf(const Polynom& poly) const {
  using namespace boost::math::tools;
  double min = poly.start;
  double max = poly.end;
  // guess is the root of the linear term of poly
  // double guess = -poly.poly[0] / poly.poly[1];
  double guess = (min + max) / 2.0;
  const int digits = std::numeric_limits<double>::digits;
  int get_digits = static_cast<int>(digits * 0.6);
  const boost::uintmax_t maxit = 20;
  boost::uintmax_t it = maxit;
  DistancePrimeFunctor functor = DistancePrimeFunctor(poly);
  return newton_raphson_iterate(functor, guess, min, max, get_digits/*, it*/);
}

Polynom ParametricSpline::getSquaredDistance(const Point& point,
                                             const Polynom2D& poly) const {
  Polynom squaredDistance;
  squaredDistance.start = poly.x.start;
  squaredDistance.end = poly.x.end;
  squaredDistance.poly = pow(point.x - poly.x.poly, 2)
      + pow(point.y - poly.y.poly, 2);
  return squaredDistance;
}

vector<Polynom2D> ParametricSpline::getPolys() const {
  vector<Polynom2D> polys;
  vector<Polynom> xpolys = getXPolys();
  vector<Polynom> ypolys = getYPolys();
  for (int i = 0; i < xpolys.size(); i++) {
    Polynom2D poly;
    poly.x = xpolys[i];
    poly.y = ypolys[i];
    polys.push_back(poly);
  }
  return polys;
}

double ParametricSpline::getDistanceOfPoly2Point(const Polynom2D& poly,
                                                 const Point& point) const {

  Polynom squaredDistance = getSquaredDistance(point, poly);
  vector<double> distances = map2<double, double>(
      { getRootOf(squaredDistance.getDerivative()), poly.x.start, poly.x.end },
      [&](double x) {return sqrt(squaredDistance(x));});
  return getMinimum(distances);
}

vector<double> ParametricSpline::getDistancesOfPolys2Point(
    const vector<Polynom2D>& polys, const Point& point) const {

  return map2<Polynom2D, double>(polys, [&](const Polynom2D& poly) {
    return getDistanceOfPoly2Point(poly, point);
  });
}

double ParametricSpline::distanceTo(const Point& point) const {
  return getMinimum(getDistancesOfPolys2Point(getPolys(), point));
}

#endif /* PARAMETRICSPLINE_H_ */
