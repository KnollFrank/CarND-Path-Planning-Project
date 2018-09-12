#include "car.h"
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

void EgoCar::setPos(const Point &pos_cart, const Frenet &pos_frenet) {
  this->pos_cart = pos_cart;
  this->pos_frenet = pos_frenet;
}

Point EgoCar::getPos_cart() const {
  return pos_cart;
}

Frenet EgoCar::getPos_frenet() const {
  return pos_frenet;
}

void EgoCar::setPos_cart(const Point &pos, const MapWaypoints &map_waypoints) {
  pos_cart = pos;
  pos_frenet = getFrenet(pos, 0, map_waypoints);
}

void EgoCar::setPos_frenet(const Frenet &pos,
                           const MapWaypoints &map_waypoints) {
  pos_frenet = pos;
  pos_cart = getXY(pos, map_waypoints);
}
