#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "coords/cart.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "dimension.h"
#include "rectangle.h"

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
  const Dimension shapeTemplate;
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
      shapeTemplate(Dimension::fromWidthAndHeight(2.5, 2.5)) {
}

void Vehicle::setPos(const FrenetCart& pos) {
  this->pos = pos;
}

FrenetCart Vehicle::getPos() const {
  return pos;
}

Rectangle Vehicle::getShape() const {
  return Rectangle::fromCenterAndDimension(getPos().getFrenet(), shapeTemplate);
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

#endif /* VEHICLE_H_ */
