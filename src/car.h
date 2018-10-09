#ifndef CAR_H_
#define CAR_H_

#include <iostream>

#include "coords/cart.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include <boost/circular_buffer.hpp>

using namespace std;

class Rectangle {

 public:
  Rectangle(const Frenet& topLeft, double width, double height);
  Frenet topLeft;
  double width;
  double height;

  Frenet bottomRight() const {
    return topLeft + Frenet { -height, width };
  }

  // TODO: refactor
  bool overlaps(const Rectangle& other) {
    const Frenet l1 = topLeft;
    const Frenet r1 = bottomRight();
    const Frenet l2 = other.topLeft;
    const Frenet r2 = other.bottomRight();

    // If one rectangle is on left side of other
    if (l1.d > r2.d || l2.d > r1.d)
      return false;

    // If one rectangle is above other
    if (l1.s < r2.s || l2.s < r1.s)
      return false;

    return true;
  }
};

// TODO: für die Kombination widht, height eine Klasse einführen, z.B. Dimension. Diese Klasse kann dann auch in EgoCar und Vehicle statt dem shapeTemplate verwendet werden.
Rectangle::Rectangle(const Frenet& _topLeft, double _width, double _height)
    : topLeft(_topLeft),
      width(_width),
      height(_height) {
}

class Circle {

 public:
  Point center;
  double radius;

  bool overlaps(const Circle& other) {
    return center.distanceTo(other.center) <= radius + other.radius;
  }
};

enum PositionHistory {
  ACTUAL = 3,
  PREVIOUS = 2,
  PREVIOUS_PREVIOUS = 1,
  PREVIOUS_PREVIOUS_PREVIOUS = 0
};

// TODO: each class within this file shall be placed into it's own file
// TODO: merge EgoCar and Vehicle, because they are essentially the same thing.
class EgoCar {

 public:
  EgoCar(const CoordsConverter& coordsConverter);
  double yaw_deg;
  double speed_mph;

  void setPos(const FrenetCart& pos);
  FrenetCart getPos() const;
  Point getAcceleration(double dt) const;
  Point getJerk(double dt) const;
  Rectangle getShape() const;

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);

 private:
  Point getVelocity(const PositionHistory& positionHistory, double dt) const;
  Point getAcceleration(const PositionHistory& positionHistory,
                        double dt) const;

  const CoordsConverter& coordsConverter;
  boost::circular_buffer<FrenetCart> positions;
  const Rectangle shapeTemplate;
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
      positions(boost::circular_buffer<FrenetCart>(4)),
      shapeTemplate(Rectangle(Frenet::zero(), 2.5, 2.5)) {
}

// TODO: DRY with Vehicle.getShape()
Rectangle EgoCar::getShape() const {
  // TODO: neuer Konstruktor mit center und width und height
  return Rectangle(
      getPos().getFrenet()
          + Frenet { shapeTemplate.height / 2, -shapeTemplate.width / 2 },
      shapeTemplate.width, shapeTemplate.height);
}

void EgoCar::setPos(const FrenetCart& pos) {
  positions.push_back(pos);
}

FrenetCart EgoCar::getPos() const {
  return positions.back();
}

Point EgoCar::getVelocity(const PositionHistory& positionHistory,
                          double dt) const {
  return (positions[positionHistory].getXY()
      - positions[positionHistory - 1].getXY()) / dt;
}

Point EgoCar::getAcceleration(const PositionHistory& positionHistory,
                              double dt) const {
  return
      positions.full() ?
          (getVelocity(positionHistory, dt)
              - getVelocity(static_cast<PositionHistory>(positionHistory - 1),
                            dt)) / dt :
          Point::zero();
}

Point EgoCar::getAcceleration(double dt) const {
  return getAcceleration(PositionHistory::ACTUAL, dt);
}

Point EgoCar::getJerk(double dt) const {
  return
      positions.full() ?
          (getAcceleration(PositionHistory::ACTUAL)
              - getAcceleration(PositionHistory::PREVIOUS)) / dt :
          Point::zero();
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

  Rectangle getShape() const;

  friend ostream& operator<<(ostream& os, const Vehicle& vehicle);
  int id;

 private:
  FrenetCart vel_m_per_s;
  FrenetCart pos;
  const CoordsConverter& coordsConverter;
  const Rectangle shapeTemplate;
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
      coordsConverter(_coordsConverter),
      shapeTemplate(Rectangle(Frenet::zero(), 2.5, 2.5)) {
}

void Vehicle::setPos(const FrenetCart& pos) {
  this->pos = pos;
}

FrenetCart Vehicle::getPos() const {
  return pos;
}

Rectangle Vehicle::getShape() const {
  return Rectangle(
      getPos().getFrenet()
          + Frenet { shapeTemplate.height / 2, -shapeTemplate.width / 2 },
      shapeTemplate.width, shapeTemplate.height);
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
