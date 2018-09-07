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

struct MapWaypoints {
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
};

struct Point {
  double x;
  double y;

  Point& operator=(const Point &point);
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

ostream& operator<<(ostream& os, const Point& point) {
  os << "Point(x = " << point.x << ", y = " << point.y << ")";
  return os;
}

struct Frenet {
  double s;
  double d;

  friend ostream& operator<<(ostream& os, const Frenet& frenet);
};

ostream& operator<<(ostream& os, const Frenet& frenet) {
  os << "Frenet(s = " << frenet.s << ", d = " << frenet.d << ")";
  return os;
}

class EgoCar {

 public:
  double yaw_deg;
  double speed;

  void setPos(const Point pos_cart, const Frenet pos_frenet);
  void setPos_cart(const Point pos, const MapWaypoints &map_waypoints);
  Point getPos_cart() const;
  void setPos_frenet(const Frenet pos, const MapWaypoints &map_waypoints);
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
  os << "  speed = " << egoCar.speed << endl;
  return os;
}

struct PreviousData {
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  Frenet end_path;
};

class Vehicle {

 public:
  void setPos(const Point pos_cart, const Frenet pos_frenet);
  void setPos_cart(const Point pos, const MapWaypoints &map_waypoints);
  Point getPos_cart() const;
  void setPos_frenet(const Frenet pos, const MapWaypoints &map_waypoints);
  Frenet getPos_frenet() const;

  int id;
  Point vel;

  friend ostream& operator<<(ostream& os, const Vehicle& vehicle);

 private:
  Point pos_cart;
  Frenet pos_frenet;
};

ostream& operator<<(ostream& os, const Vehicle& vehicle) {
  os << "Vehicle(" << vehicle.id << "):" << endl;
  os << "  pos_cart = " << vehicle.pos_cart << endl;
  os << "  (vx, vy) = (" << vehicle.vel.x << ", " << vehicle.vel.y << ")"
     << endl;
  os << "  pos_frenet = " << vehicle.pos_frenet << endl;
  return os;
}

struct Points {
  vector<double> xs;
  vector<double> ys;
};

struct ReferencePoint {
  Point point;
  double yaw_rad;
  double vel;  // [mph]
};

#endif /* MAIN_H_ */
