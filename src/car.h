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
  Frenet getAcceleration() const;
  Frenet getJerk() const;

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);

  static double carRadius() {
    return 1.25;
  }

  static double carSize() {
    return 2 * carRadius();
  }

 private:
  Frenet getVelocity(const PositionHistory& positionHistory) const;
  Frenet getAcceleration(const PositionHistory& positionHistory) const;

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

Frenet EgoCar::getVelocity(const PositionHistory& positionHistory) const {
  return (positions[positionHistory].getFrenet(coordsConverter)
      - positions[positionHistory - 1].getFrenet(coordsConverter)) / dt;
}

Frenet EgoCar::getAcceleration(const PositionHistory& positionHistory) const {
  return
      positions.full() ?
          (getVelocity(positionHistory)
              - getVelocity(static_cast<PositionHistory>(positionHistory - 1)))
              / dt :
          Frenet::zero();
}

// TODO: when computing the acceleration in cartesian coordinates (which would be the correct way) we always exceed 10 m/s^2, so I shifted to the wrong way of using Frenet coordinates.
Frenet EgoCar::getAcceleration() const {
  return getAcceleration(PositionHistory::ACTUAL);
}

Frenet EgoCar::getJerk() const {
  return
      positions.full() ?
          (getAcceleration(PositionHistory::ACTUAL)
              - getAcceleration(PositionHistory::PREVIOUS)) / dt :
          Frenet::zero();
}

class Vehicle {

 public:
  Vehicle(int id, const CoordsConverter& coordsConverter);
  void setPos(const FrenetCart& pos);
  FrenetCart getPos() const;

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

Vehicle::Vehicle(int _id, const CoordsConverter& _coordsConverter)
    : id(_id),
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
      coordsConverter.createFrenetVectorFromStart2End(
          getPos().getXY(coordsConverter),
          getPos().getXY(coordsConverter) + vel),
      vel);
}

void Vehicle::setVel_frenet_m_per_s(const Frenet& vel) {
  vel_m_per_s = FrenetCart(
      vel,
      coordsConverter.createCartVectorFromStart2End(
          getPos().getFrenet(coordsConverter),
          getPos().getFrenet(coordsConverter) + vel));
}

Point Vehicle::getVel_cart_m_per_s() const {
  return vel_m_per_s.getXY(coordsConverter);
}

Frenet Vehicle::getVel_frenet_m_per_s() const {
  return vel_m_per_s.getFrenet(coordsConverter);
}

#endif /* CAR_H_ */
