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

class ParametricSpline {

 public:
  ParametricSpline(const real_2d_array &xy, const SplineType st,
                   const ParameterizationType pt);

 // private:
  pspline2interpolant spline;
};

ParametricSpline::ParametricSpline(const real_2d_array &xy, const SplineType st,
                                   const ParameterizationType pt) {
  pspline2buildperiodic(xy, xy.rows(), st, pt, spline);
}

#endif /* PARAMETRICSPLINE_H_ */
