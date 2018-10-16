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
#include "../funs.h"

using namespace std;

struct Frenet {
  double s;
  double d;

  static Frenet fromAngle(double angle_rad);
  static Frenet zero();
  Frenet operator+(const Frenet& other) const;
  Frenet operator-(const Frenet& other) const;
  Frenet minusCircular(const Frenet& prev, const double len) const;
  Frenet operator*(double scalar) const;
  Frenet operator/(double scalar) const;

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

Frenet Frenet::minusCircular(const Frenet& prev, const double len) const {
  const Frenet diff = *this - prev;
  return prev.s <= s ? diff : diff + Frenet { len, 0 };
}

Frenet Frenet::operator*(double scalar) const {
  return Frenet { s * scalar, d * scalar };
}

Frenet Frenet::operator/(double scalar) const {
  return *this * (1.0 / scalar);
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

Frenet Frenet::zero() {
  return Frenet { 0, 0 };
}

double Frenet::getHeading() const {
  return atan2(d, s);
}

double Frenet::len() const {
  return distanceTo(zero());
}

double Frenet::distanceTo(const Frenet &point) const {
  Frenet diff = point - *this;
  return sqrt(diff.scalarProd(diff));
}

double Frenet::scalarProd(const Frenet &point) const {
  return s * point.s + d * point.d;
}

#endif /* COORDS_FRENET_H_ */
