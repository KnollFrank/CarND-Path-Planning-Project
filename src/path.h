#ifndef PATH_H_
#define PATH_H_

#include <vector>
#include <tuple>
#include "coords/cart.h"
#include "coords/frenet.h"
#include "coords/coordsConverter.h"
#include "spline.h"
#include <experimental/optional>

using namespace std;
using namespace std::experimental;

class FrenetCart {
 public:
  FrenetCart();
  FrenetCart(std::experimental::optional<Frenet> frenet,
             std::experimental::optional<Point> cart);

  // TODO: add constructors for either Frenet or Point
  Frenet getFrenet(const CoordsConverter& coordsConverter) const;
  Point getXY(const CoordsConverter& coordsConverter) const;

 private:
  std::experimental::optional<Frenet> frenet;
  std::experimental::optional<Point> cart;
};

FrenetCart::FrenetCart()
    : FrenetCart(std::experimental::nullopt, std::experimental::nullopt) {
}

FrenetCart::FrenetCart(std::experimental::optional<Frenet> _frenet,
                       std::experimental::optional<Point> _cart)
    : frenet(_frenet),
      cart(_cart) {
}

Frenet FrenetCart::getFrenet(const CoordsConverter& coordsConverter) const {
  return frenet ? frenet.value() : coordsConverter.getFrenet(cart.value());
}

Point FrenetCart::getXY(const CoordsConverter& coordsConverter) const {
  return cart ? cart.value() : coordsConverter.getXY(frenet.value());
}

struct Path {
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
