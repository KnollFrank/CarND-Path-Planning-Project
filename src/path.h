#ifndef PATH_H_
#define PATH_H_

#include <vector>

#include "coords/cart.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "funs.h"
#include "spline.h"

using namespace std;
using namespace std::experimental;

class Path {
 public:
  vector<FrenetCart> points;

  vector<double> asXVals(const CoordsConverter& coordsConverter) const;
  vector<double> asYVals(const CoordsConverter& coordsConverter) const;
  tk::spline asSpline(const CoordsConverter& coordsConverter) const;
  friend ostream& operator<<(ostream& os, const Path& path);

 private:
  vector<Point> asPoints(const CoordsConverter& coordsConverter) const;
  vector<Frenet> asFrenets(const CoordsConverter& coordsConverter) const;
  vector<double> asSVals(const CoordsConverter& coordsConverter) const;
  vector<double> asDVals(const CoordsConverter& coordsConverter) const;
};

ostream& operator<<(ostream& os, const Path& path) {
  os << "Path:" << endl;
  os << "  points = " << path.points << endl;
  return os;
}

vector<Point> Path::asPoints(const CoordsConverter& coordsConverter) const {
  return map2<FrenetCart, Point>(points, [&](const FrenetCart& point) {
    return point.getXY(coordsConverter);
  });
}

vector<Frenet> Path::asFrenets(const CoordsConverter& coordsConverter) const {
  return map2<FrenetCart, Frenet>(points, [&](const FrenetCart& point) {
    return point.getFrenet(coordsConverter);
  });
}

vector<double> Path::asXVals(const CoordsConverter& coordsConverter) const {
  return map2<Point, double>(asPoints(coordsConverter),
                             [](const Point& point) {return point.x;});
}

vector<double> Path::asYVals(const CoordsConverter& coordsConverter) const {
  return map2<Point, double>(asPoints(coordsConverter),
                             [](const Point& point) {return point.y;});
}

vector<double> Path::asSVals(const CoordsConverter& coordsConverter) const {
  return map2<Frenet, double>(asFrenets(coordsConverter),
                              [](const Frenet& point) {return point.s;});
}

vector<double> Path::asDVals(const CoordsConverter& coordsConverter) const {
  return map2<Frenet, double>(asFrenets(coordsConverter),
                              [](const Frenet& point) {return point.d;});
}

tk::spline Path::asSpline(const CoordsConverter& coordsConverter) const {
  tk::spline splines;
  splines.set_points(asSVals(coordsConverter), asDVals(coordsConverter));
  return splines;
}

#endif /* PATH_H_ */
