#ifndef SPLINE_H_
#define SPLINE_H_

#include <boost/math/tools/polynomial.hpp>
#include <boost/math/tools/roots.hpp>
#include <cmath>
#include <csetjmp>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "alglib/ap.h"
#include "alglib/interpolation.h"
#include "coords/cart.h"
#include "coords/frenet.h"
#include "funs.h"
#include <experimental/optional>

using namespace std::experimental;
using namespace alglib;
using namespace boost::math::tools;

class Spline {

 public:
  Spline(const vector<double> &xs, const vector<double> &ys);

  double operator()(double x) const;
  double toSplineParameter(double s) const;

 private:
  spline1dinterpolant spline;
};

Spline::Spline(const vector<double> &xs, const vector<double> &ys) {
  real_1d_array x;
  x.setcontent(xs.size(), &(xs[0]));

  real_1d_array y;
  y.setcontent(ys.size(), &(ys[0]));

  spline1dbuildcubic(x, y, spline);
}

double Spline::operator()(double x) const {
  return spline1dcalc(spline, x);
}

#endif /* SPLINE_H_ */
