#ifndef PATH_H_
#define PATH_H_

#include <vector>
#include "coords/cart.h"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

struct Path {
  vector<Point> points;

  tuple<vector<double>, vector<double>> asXValsAndYVals() const;
  tk::spline asSpline() const;
};

tuple<vector<double>, vector<double>> Path::asXValsAndYVals() const {
  vector<double> xs;
  vector<double> ys;
  for (const Point& point : points) {
    xs.push_back(point.x);
    ys.push_back(point.y);
  }

  return make_tuple(xs, ys);
}

tk::spline Path::asSpline() const {
  vector<double> xs;
  vector<double> ys;
  tie(xs, ys) = asXValsAndYVals();
  tk::spline s;
  s.set_points(xs, ys);
  return s;
}

#endif /* PATH_H_ */
