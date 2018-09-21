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
  // TODO: "vector<tuple<optional<Frenet>, optional<Point>>> points". Falls einer der beiden Werte im Tupel nicht vorhanden ist, soll er aus dem anderen Wert berechnet werden.
  vector<Frenet> points;

  tuple<vector<double>, vector<double>> asSValsAndDVals() const;
  tuple<vector<double>, vector<double>> asXValsAndYVals(const CoordsConverter& coordsConverter) const;
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

vector<Point> asPoints(const vector<Frenet>& points, const CoordsConverter& coordsConverter) {
  return map2<Frenet, Point>(points, [&](const Frenet& point) {
    return coordsConverter.getXY(point);
  });
}

tuple<vector<double>, vector<double>> Path::asXValsAndYVals(const CoordsConverter& coordsConverter) const {
  vector<double> xs;
  vector<double> ys;
  for (const Point& point : asPoints(points, coordsConverter)) {
    xs.push_back(point.x);
    ys.push_back(point.y);
  }

  return make_tuple(xs, ys);
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
