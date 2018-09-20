#ifndef CAR_H_
#define CAR_H_

#include "coords/coordsConverter.h"

using namespace std;

// TODO: merge EgoCar and Vehicle, because they are essentially the same thing.
class EgoCar {

 public:
  EgoCar(const CoordsConverter& coordsConverter);
  double yaw_deg;
  double speed_mph;

  void setPos(const Point& pos_cart, const Frenet& pos_frenet);

  void setPos_cart(const Point& pos);
  Point getPos_cart() const;

  void setPos_frenet(const Frenet& pos);
  Frenet getPos_frenet() const;

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);

  static double carRadius() {
    return 1.25;
  }
  static double carSize() {
    return 2 * carRadius();
  }

 private:
  Point pos_cart;
  Frenet pos_frenet;
  const CoordsConverter& coordsConverter;
};

ostream& operator<<(ostream& os, const EgoCar& egoCar) {
  os << "EgoCar:" << endl;
  os << "  pos_frenet = " << egoCar.pos_frenet << endl;
  os << "  yaw = " << egoCar.yaw_deg << "°" << endl;
  os << "  speed = " << egoCar.speed_mph << " mph" << endl;
  return os;
}

EgoCar::EgoCar(const CoordsConverter& _coordsConverter)
    : coordsConverter(_coordsConverter) {
}

void EgoCar::setPos(const Point& pos_cart, const Frenet& pos_frenet) {
  this->pos_cart = pos_cart;
  this->pos_frenet = pos_frenet;
}

Point EgoCar::getPos_cart() const {
  return pos_cart;
}

Frenet EgoCar::getPos_frenet() const {
  return pos_frenet;
}

void EgoCar::setPos_cart(const Point& pos) {
  pos_cart = pos;
  pos_frenet = coordsConverter.getFrenet(pos);
}

void EgoCar::setPos_frenet(const Frenet& pos) {
  pos_frenet = pos;
  pos_cart = coordsConverter.getXY(pos);
}

class Vehicle {

 public:
  Vehicle(const CoordsConverter& coordsConverter);
  void setPos(const Point& pos_cart, const Frenet& pos_frenet);

  void setPos_cart(const Point& pos);
  Point getPos_cart() const;

  void setPos_frenet(const Frenet& pos);
  Frenet getPos_frenet() const;

  void setVel_cart_m_per_s(const Point& vel);
  Point getVel_cart_m_per_s() const;

  void setVel_frenet_m_per_s(const Frenet& vel);
  Frenet getVel_frenet_m_per_s() const;

  int id;

  friend ostream& operator<<(ostream& os, const Vehicle& vehicle);

 private:
  Point vel_cart_m_per_s;
  Frenet vel_frenet_m_per_s;
  Point pos_cart;
  Frenet pos_frenet;
  const CoordsConverter& coordsConverter;
};

ostream& operator<<(ostream& os, const Vehicle& vehicle) {
  os << "Vehicle(" << vehicle.id << "):" << endl;
  os << "  pos_frenet = " << vehicle.pos_frenet << endl;
  os << "  vel_frenet_m_per_s = " << vehicle.vel_frenet_m_per_s << " m/s" << endl;
  return os;
}

Vehicle::Vehicle(const CoordsConverter& _coordsConverter)
    : coordsConverter(_coordsConverter) {
}

void Vehicle::setPos(const Point& pos_cart, const Frenet& pos_frenet) {
  this->pos_cart = pos_cart;
  this->pos_frenet = pos_frenet;
}

Point Vehicle::getPos_cart() const {
  return pos_cart;
}

Frenet Vehicle::getPos_frenet() const {
  return pos_frenet;
}

void Vehicle::setVel_cart_m_per_s(const Point& vel) {
  vel_cart_m_per_s = vel;
  vel_frenet_m_per_s = coordsConverter.createFrenetVectorFromStart2End(
      getPos_cart(), getPos_cart() + vel);
}

void Vehicle::setVel_frenet_m_per_s(const Frenet& vel) {
  vel_frenet_m_per_s = vel;
  vel_cart_m_per_s = coordsConverter.createCartVectorFromStart2End(
      getPos_frenet(), getPos_frenet() + vel);
}

Point Vehicle::getVel_cart_m_per_s() const {
  return vel_cart_m_per_s;
}

Frenet Vehicle::getVel_frenet_m_per_s() const {
  return vel_frenet_m_per_s;
}

void Vehicle::setPos_cart(const Point& pos) {
  pos_cart = pos;
  pos_frenet = coordsConverter.getFrenet(pos);
}

void Vehicle::setPos_frenet(const Frenet& pos) {
  pos_frenet = pos;
  pos_cart = coordsConverter.getXY(pos);
}

#endif /* CAR_H_ */
