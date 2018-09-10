#ifndef MAIN_H_
#define MAIN_H_

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

using namespace std;

struct Point {
  double x;
  double y;

  static Point fromAngle(double angle_rad);
  double len() const;
  Point& operator=(const Point &point);
  Point operator+(const Point& other) const;
  Point operator-(const Point& other) const;
  Point operator*(double scalar) const;

  friend ostream& operator<<(ostream& os, const Point& point);
};

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

struct MapWaypoints {
  vector<Point> map_waypoints;
  vector<double> map_waypoints_s;
};

double scalarProd(const Point &a, const Point &b) {
  return a.x * b.x + a.y * b.y;
}

ostream& operator<<(ostream& os, const Point& point) {
  os << "Point(x = " << point.x << ", y = " << point.y << ")";
  return os;
}

struct Frenet {
  double s;
  double d;

  Frenet operator+(const Frenet& other) const;
  Frenet operator-(const Frenet& other) const;
  Frenet operator*(double scalar) const;

  friend ostream& operator<<(ostream& os, const Frenet& frenet);
};

ostream& operator<<(ostream& os, const Frenet& frenet) {
  os << "Frenet(s = " << frenet.s << ", d = " << frenet.d << ")";
  return os;
}

// TODO: merge EgoCar and Vehicle, because they are essentially the same thing.
class EgoCar {

 public:
  double yaw_deg;
  double speed_mph;

  void setPos(const Point &pos_cart, const Frenet &pos_frenet);

  void setPos_cart(const Point &pos, const MapWaypoints &map_waypoints);
  Point getPos_cart() const;

  void setPos_frenet(const Frenet &pos, const MapWaypoints &map_waypoints);
  Frenet getPos_frenet() const;

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);

 private:
  Point pos_cart;
  Frenet pos_frenet;
};

ostream& operator<<(ostream& os, const EgoCar& egoCar) {
  os << "EgoCar:" << endl;
  os << "  pos_cart = " << egoCar.pos_cart << endl;
  os << "  pos_frenet = " << egoCar.pos_frenet << endl;
  os << "  yaw = " << egoCar.yaw_deg << "°" << endl;
  os << "  speed = " << egoCar.speed_mph << " mph" << endl;
  return os;
}

class Vehicle {

 public:
  void setPos(const Point &pos_cart, const Frenet &pos_frenet);

  void setPos_cart(const Point &pos, const MapWaypoints &map_waypoints);
  Point getPos_cart() const;

  void setPos_frenet(const Frenet &pos, const MapWaypoints &map_waypoints);
  Frenet getPos_frenet() const;

  void setVel_cart_m_per_s(const Point &vel);
  Point getVel_cart_m_per_s() const;

  void setVel_frenet_m_per_s(const Frenet &vel,
                             const MapWaypoints &map_waypoints);
  Frenet getVel_frenet_m_per_s(const MapWaypoints &map_waypoints) const;

  int id;

  friend ostream& operator<<(ostream& os, const Vehicle& vehicle);

 private:
  Point vel_cart_m_per_s;
  Point pos_cart;
  Frenet pos_frenet;
};

ostream& operator<<(ostream& os, const Vehicle& vehicle) {
  os << "Vehicle(" << vehicle.id << "):" << endl;
  os << "  pos_cart = " << vehicle.pos_cart << endl;
  os << "  (vx, vy) = (" << vehicle.vel_cart_m_per_s.x << " m/s, "
     << vehicle.vel_cart_m_per_s.y << " m/s)" << endl;
  os << "  pos_frenet = " << vehicle.pos_frenet << endl;
  return os;
}

struct Path {
  vector<Point> points;
};

struct PreviousData {
  Path previous_path;
  Frenet end_path;
};

struct ReferencePoint {
  Point point;
  double yaw_rad;
  double vel_mph;
};

#endif /* MAIN_H_ */
