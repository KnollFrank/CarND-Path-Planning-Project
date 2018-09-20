#ifndef PATH_H_
#define PATH_H_

#include <vector>
#include <tuple>
#include "coords/cart.h"
#include "coords/frenet.h"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

struct Path {
  vector<Frenet> points;

  tuple<vector<double>, vector<double>> asSValsAndDVals() const;
  tk::spline asSpline() const;
};

tuple<vector<double>, vector<double>> Path::asSValsAndDVals() const {
  vector<double> ss;
  vector<double> ds;
  for (const Frenet& point : points) {
    ss.push_back(point.s);
    ds.push_back(point.d);
  }

  return make_tuple(ss, ds);
}

tk::spline Path::asSpline() const {
  vector<double> ss;
  vector<double> ds;
  tie(ss, ds) = asSValsAndDVals();
  tk::spline s;
  s.set_points(ss, ds);
  return s;
}

#endif /* PATH_H_ */
