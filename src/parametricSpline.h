#ifndef PARAMETRICSPLINE_H_
#define PARAMETRICSPLINE_H_

#include "alglib/stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
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
                                   pspline2interpolant &p) {
  alglib::ae_int_t n = xy.rows();
  pspline2buildperiodic(xy, n, st, pt, p);
}

#endif /* PARAMETRICSPLINE_H_ */
