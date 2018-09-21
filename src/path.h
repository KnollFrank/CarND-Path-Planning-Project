#ifndef PATH_H_
#define PATH_H_

#include <vector>
#include <tuple>
#include "coords/cart.h"
#include "coords/frenet.h"
#include "coords/coordsConverter.h"
#include "spline.h"
#include <experimental/optional>
#include "coords/frenetCart.h"

using namespace std;
using namespace std::experimental;

class Path {
 public:
  vector<FrenetCart> points;

  tuple<vector<double>, vector<double>> asSValsAndDVals(
      const CoordsConverter& coordsConverter) const;
  tuple<vector<double>, vector<double>> asXValsAndYVals(
      const CoordsConverter& coordsConverter) const;
  tk::spline asSpline(const CoordsConverter& coordsConverter) const;
};

tuple<vector<double>, vector<double>> Path::asSValsAndDVals(
    const CoordsConverter& coordsConverter) const {
  vector<double> ss;
  vector<double> ds;
  for (const FrenetCart& point : points) {
    ss.push_back(point.getFrenet(coordsConverter).s);
    ds.push_back(point.getFrenet(coordsConverter).d);
  }

  return make_tuple(ss, ds);
}

vector<Point> asPoints(const vector<FrenetCart>& points,
                       const CoordsConverter& coordsConverter) {
  return map2<FrenetCart, Point>(points, [&](const FrenetCart& point) {
    return point.getXY(coordsConverter);
  });
}

tuple<vector<double>, vector<double>> Path::asXValsAndYVals(
    const CoordsConverter& coordsConverter) const {
  vector<double> xs;
  vector<double> ys;
  for (const Point& point : asPoints(points, coordsConverter)) {
    xs.push_back(point.x);
    ys.push_back(point.y);
  }

  return make_tuple(xs, ys);
}

tk::spline Path::asSpline(const CoordsConverter& coordsConverter) const {
  vector<double> ss;
  vector<double> ds;
  tie(ss, ds) = asSValsAndDVals(coordsConverter);
  tk::spline s;
  s.set_points(ss, ds);
  return s;
}

#endif /* PATH_H_ */
