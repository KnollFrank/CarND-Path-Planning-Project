#ifndef PARAMETRICSPLINE_H_
#define PARAMETRICSPLINE_H_

#include "alglib/ap.h"
#include "alglib/interpolation.h"

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

void buildPeriodicParametricSpline(const real_2d_array &xy, const SplineType st,
                                   const ParameterizationType pt,
                                   pspline2interpolant &spline) {
  alglib::ae_int_t n = xy.rows();
  pspline2buildperiodic(xy, n, st, pt, spline);
}

#endif /* PARAMETRICSPLINE_H_ */
