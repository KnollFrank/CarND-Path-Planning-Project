#ifndef FRENET_H_
#define FRENET_H_

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

struct Frenet {
  double s;
  double d;

  Frenet operator+(const Frenet& other) const;
  Frenet operator-(const Frenet& other) const;
  Frenet operator*(double scalar) const;

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

#endif /* FRENET_H_ */
