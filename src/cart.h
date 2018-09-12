#ifndef CART_H_
#define CART_H_

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <tuple>
#include "mathfuns.h"

using namespace std;

struct Point {
  double x;
  double y;

  static Point fromAngle(double angle_rad);
  double len() const;
  double scalarProd(const Point &point) const;
  double distanceTo(const Point &point) const;
  double getHeading() const;
  Point& operator=(const Point &point);
  Point operator+(const Point& other) const;
  Point operator-(const Point& other) const;
  Point operator*(double scalar) const;

  friend ostream& operator<<(ostream& os, const Point& point);
};

Point Point::fromAngle(double angle_rad) {
  return Point { cos(angle_rad), sin(angle_rad) };
}

double Point::len() const {
  return distanceTo(Point { 0, 0 });
}

double Point::scalarProd(const Point &point) const {
  return x * point.x + y * point.y;
}

double Point::distanceTo(const Point &point) const {
  Point diff = point - *this;
  return sqrt(diff.scalarProd(diff));
}

double Point::getHeading() const {
  return atan2(y, x);
}

Point Point::operator+(const Point &other) const {
  return Point { x + other.x, y + other.y };
}

Point Point::operator-(const Point &other) const {
  return *this + (other * -1);
}

Point Point::operator*(double scalar) const {
  return Point { x * scalar, y * scalar };
}

Point& Point::operator=(const Point &point) {
  // self-assignment guard
  if (this == &point)
    return *this;

  // do the copy
  x = point.x;
  y = point.y;

  // return the existing object so we can chain this operator
  return *this;
}

#endif /* CART_H_ */
