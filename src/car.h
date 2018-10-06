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
  EgoCar(const CoordsConverter& coordsConverter);
  double yaw_deg;
  double speed_mph;

  void setPos(const FrenetCart& pos);
  FrenetCart getPos() const;
  Frenet getAcceleration(double dt) const;
  Frenet getJerk(double dt) const;

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);

  static double carRadius() {
    return 1.25;
  }

  static double carSize() {
    return 2 * carRadius();
  }

 private:
  Frenet getVelocity(const PositionHistory& positionHistory, double dt) const;
  Frenet getAcceleration(const PositionHistory& positionHistory,
                         double dt) const;

  const CoordsConverter& coordsConverter;
  boost::circular_buffer<FrenetCart> positions;
};

ostream& operator<<(ostream& os, const EgoCar& egoCar) {
  os << "EgoCar:" << endl;
  os << "  position = " << egoCar.getPos();
  os << "  yaw = " << egoCar.yaw_deg << "°" << endl;
  os << "  speed = " << egoCar.speed_mph << " mph ("
     << mph2meter_per_sec(egoCar.speed_mph) << " m/s)" << endl;
  return os;
}

EgoCar::EgoCar(const CoordsConverter& _coordsConverter)
    : coordsConverter(_coordsConverter),
      positions(boost::circular_buffer<FrenetCart>(4)) {
}

void EgoCar::setPos(const FrenetCart& pos) {
  positions.push_back(pos);
}

FrenetCart EgoCar::getPos() const {
  return positions.back();
}

Frenet EgoCar::getVelocity(const PositionHistory& positionHistory,
                           double dt) const {
  return (positions[positionHistory].getFrenet()
      - positions[positionHistory - 1].getFrenet()) / dt;
}

Frenet EgoCar::getAcceleration(const PositionHistory& positionHistory,
                               double dt) const {
  return
      positions.full() ?
          (getVelocity(positionHistory, dt)
              - getVelocity(static_cast<PositionHistory>(positionHistory - 1),
                            dt)) / dt :
          Frenet::zero();
}

// TODO: when computing the acceleration in cartesian coordinates (which would be the correct way) we always exceed 10 m/s^2, so I shifted to the wrong way of using Frenet coordinates.
Frenet EgoCar::getAcceleration(double dt) const {
  return getAcceleration(PositionHistory::ACTUAL, dt);
}

Frenet EgoCar::getJerk(double dt) const {
  return
      positions.full() ?
          (getAcceleration(PositionHistory::ACTUAL)
              - getAcceleration(PositionHistory::PREVIOUS)) / dt :
          Frenet::zero();
}

class Vehicle {

 public:
  Vehicle(int id, FrenetCart pos, const CoordsConverter& coordsConverter);
  void setPos(const FrenetCart& pos);
  FrenetCart getPos() const;

  void setVel_cart_m_per_s(const Point& vel);
  Point getVel_cart_m_per_s() const;

  void setVel_frenet_m_per_s(const Frenet& vel);
  Frenet getVel_frenet_m_per_s() const;

  friend ostream& operator<<(ostream& os, const Vehicle& vehicle);
  int id;

 private:
  FrenetCart vel_m_per_s;
  FrenetCart pos;
  const CoordsConverter& coordsConverter;
};

ostream& operator<<(ostream& os, const Vehicle& vehicle) {
  os << "Vehicle(" << vehicle.id << "):" << endl;
  os << "  pos = " << vehicle.pos;
  os << "  vel_m_per_s = " << vehicle.vel_m_per_s << " m/s" << endl;
  return os;
}

Vehicle::Vehicle(int _id, FrenetCart _pos,
                 const CoordsConverter& _coordsConverter)
    : id(_id),
      pos(_pos),
      vel_m_per_s(FrenetCart(Point::zero(), _coordsConverter)),
      coordsConverter(_coordsConverter) {
}

void Vehicle::setPos(const FrenetCart& pos) {
  this->pos = pos;
}

FrenetCart Vehicle::getPos() const {
  return pos;
}

void Vehicle::setVel_cart_m_per_s(const Point& vel) {
  vel_m_per_s = FrenetCart(
      coordsConverter.createFrenetVectorFromStart2End(getPos().getXY(),
                                                      getPos().getXY() + vel),
      vel, coordsConverter);
}

void Vehicle::setVel_frenet_m_per_s(const Frenet& vel) {
  vel_m_per_s = FrenetCart(
      vel,
      coordsConverter.createCartVectorFromStart2End(getPos().getFrenet(),
                                                    getPos().getFrenet() + vel),
      coordsConverter);
}

Point Vehicle::getVel_cart_m_per_s() const {
  return vel_m_per_s.getXY();
}

Frenet Vehicle::getVel_frenet_m_per_s() const {
  return vel_m_per_s.getFrenet();
}

#endif /* CAR_H_ */
