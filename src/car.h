#ifndef CAR_H_
#define CAR_H_

#include <iostream>

#include "coords/cart.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include <boost/circular_buffer.hpp>

using namespace std;

enum PositionHistory {
  ACTUAL = 3,
  PREVIOUS = 2,
  PREVIOUS_PREVIOUS = 1,
  PREVIOUS_PREVIOUS_PREVIOUS = 0
};

// TODO: merge EgoCar and Vehicle, because they are essentially the same thing.
class EgoCar {

 public:
  EgoCar(const CoordsConverter& coordsConverter, double dt);
  double yaw_deg;
  double speed_mph;

  void setPos(const FrenetCart& pos);
  FrenetCart getPos() const;
  Frenet getAcceleration();

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);

  static double carRadius() {
    return 1.25;
  }

  static double carSize() {
    return 2 * carRadius();
  }

 private:
  Frenet getVelocity(PositionHistory positionHistory) const;

  const CoordsConverter& coordsConverter;
  boost::circular_buffer<FrenetCart> positions;
  double dt;
};

ostream& operator<<(ostream& os, const EgoCar& egoCar) {
  os << "EgoCar:" << endl;
  os << "  positions = " << egoCar.positions << endl;
  os << "  yaw = " << egoCar.yaw_deg << "°" << endl;
  os << "  speed = " << egoCar.speed_mph << " mph" << endl;
  return os;
}

EgoCar::EgoCar(const CoordsConverter& _coordsConverter, double _dt)
    : coordsConverter(_coordsConverter),
      positions(boost::circular_buffer<FrenetCart>(4)),
      dt(_dt) {
}

void EgoCar::setPos(const FrenetCart& pos) {
  positions.push_back(pos);
}

FrenetCart EgoCar::getPos() const {
  return positions.back();
}

Frenet EgoCar::getVelocity(PositionHistory positionHistory) const {
  return (positions[positionHistory].getFrenet(coordsConverter)
      - positions[positionHistory - 1].getFrenet(coordsConverter)) / dt;
}

// TODO: when computing the acceleration in cartesian coordinates (which would be the correct way) we always exceed 10 m/s^2, so I shifted to the wrong way of using Frenet coordinates.
Frenet EgoCar::getAcceleration() {
  return
      positions.full() ?
          (getVelocity(PositionHistory::ACTUAL)
              - getVelocity(PositionHistory::PREVIOUS)) / dt :
          Frenet::zero();
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
  FrenetCart vel_m_per_s;
  FrenetCart pos;
  const CoordsConverter& coordsConverter;
};

ostream& operator<<(ostream& os, const Vehicle& vehicle) {
  os << "Vehicle(" << vehicle.id << "):" << endl;
  os << "  pos_frenet = " << vehicle.pos << endl;
  os << "  vel_m_per_s = " << vehicle.vel_m_per_s << " m/s" << endl;
  return os;
}

Vehicle::Vehicle(const CoordsConverter& _coordsConverter)
    : coordsConverter(_coordsConverter) {
}

void Vehicle::setPos(const Point& pos_cart, const Frenet& pos_frenet) {
  pos = FrenetCart(pos_frenet, pos_cart);
}

Point Vehicle::getPos_cart() const {
  return pos.getXY(coordsConverter);
}

Frenet Vehicle::getPos_frenet() const {
  return pos.getFrenet(coordsConverter);
}

void Vehicle::setVel_cart_m_per_s(const Point& vel) {
  vel_m_per_s = FrenetCart(
      coordsConverter.createFrenetVectorFromStart2End(getPos_cart(),
                                                      getPos_cart() + vel),
      vel);
}

void Vehicle::setVel_frenet_m_per_s(const Frenet& vel) {
  vel_m_per_s = FrenetCart(
      vel,
      coordsConverter.createCartVectorFromStart2End(getPos_frenet(),
                                                    getPos_frenet() + vel));
}

Point Vehicle::getVel_cart_m_per_s() const {
  return vel_m_per_s.getXY(coordsConverter);
}

Frenet Vehicle::getVel_frenet_m_per_s() const {
  return vel_m_per_s.getFrenet(coordsConverter);
}

void Vehicle::setPos_cart(const Point& pos) {
  this->pos = FrenetCart(pos);
}

void Vehicle::setPos_frenet(const Frenet& pos) {
  this->pos = FrenetCart(pos);
}

#endif /* CAR_H_ */
