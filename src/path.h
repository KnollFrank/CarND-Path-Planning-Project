#ifndef PATH_H_
#define PATH_H_

#include <iostream>
#include <vector>

#include "coords/cart.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "coords/waypoints.h"
#include "funs.h"
#include "spline.h"

using namespace std;
using namespace std::experimental;

class Path {
 public:
  vector<FrenetCart> points;

  vector<double> asXVals() const;
  vector<double> asYVals() const;
  Spline asXYSpline() const;
  double getCartLen() const;
  friend ostream& operator<<(ostream& os, const Path& path);

 private:
  vector<Point> asPoints() const;
  vector<Frenet> asFrenets() const;
  vector<double> asSVals() const;
  vector<double> asDVals() const;
};

ostream& operator<<(ostream& os, const Path& path) {
  os << "Path:" << endl;
  os << "  points = " << path.points << endl;
  return os;
}

vector<Point> Path::asPoints() const {
  return map2<FrenetCart, Point>(points, [&](const FrenetCart& point) {
    return point.getXY();
  });
}

vector<Frenet> Path::asFrenets() const {
  return map2<FrenetCart, Frenet>(points, [&](const FrenetCart& point) {
    return point.getFrenet();
  });
}

vector<double> Path::asXVals() const {
  return map2<Point, double>(asPoints(),
                             [](const Point& point) {return point.x;});
}

vector<double> Path::asYVals() const {
  return map2<Point, double>(asPoints(),
                             [](const Point& point) {return point.y;});
}

vector<double> Path::asSVals() const {
  return map2<Frenet, double>(asFrenets(),
                              [](const Frenet& point) {return point.s;});
}

vector<double> Path::asDVals() const {
  return map2<Frenet, double>(asFrenets(),
                              [](const Frenet& point) {return point.d;});
}

Spline Path::asXYSpline() const {
  Spline spline(asXVals(), asYVals());
  return spline;
}

double Path::getCartLen() const {
  return getLenOfTraverse(asPoints());
}

#endif /* PATH_H_ */
