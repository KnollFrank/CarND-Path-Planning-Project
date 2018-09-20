#ifndef COORDS_FRENET_H_
#define COORDS_FRENET_H_

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>
#include "../mathfuns.h"

using namespace std;

struct Frenet {
  double s;
  double d;

  static Frenet fromAngle(double angle_rad);
  Frenet operator+(const Frenet& other) const;
  Frenet operator-(const Frenet& other) const;
  Frenet operator*(double scalar) const;

  double getHeading() const;
  double len() const;
  double distanceTo(const Frenet &point) const;
  double scalarProd(const Frenet &point) const;

  friend ostream& operator<<(ostream& os, const Frenet& frenet);
};

Frenet Frenet::operator+(const Frenet &other) const {
  return Frenet { s + other.s, d + other.d };
}

Frenet Frenet::operator-(const Frenet &other) const {
  return *this + (other * -1);
}

Frenet Frenet::operator*(double scalar) const {
  return Frenet { s * scalar, d * scalar };
}

ostream& operator<<(ostream& os, const Point& point) {
  os << "Point(x = " << point.x << ", y = " << point.y << ")";
  return os;
}

ostream& operator<<(ostream& os, const Frenet& frenet) {
  os << "Frenet(s = " << frenet.s << ", d = " << frenet.d << ")";
  return os;
}

bool operator==(const Frenet& lhs, const Frenet& rhs) {
  return lhs.s == rhs.s && lhs.d == rhs.d;
}

Frenet Frenet::fromAngle(double angle_rad) {
  return Frenet { cos(angle_rad), sin(angle_rad) };
}

double Frenet::getHeading() const {
  return atan2(d, s);
}

double Frenet::len() const {
  return distanceTo(Frenet { 0, 0 });
}

double Frenet::distanceTo(const Frenet &point) const {
  Frenet diff = point - *this;
  return sqrt(diff.scalarProd(diff));
}

double Frenet::scalarProd(const Frenet &point) const {
  return s * point.s + d * point.d;
}

#endif /* COORDS_FRENET_H_ */
