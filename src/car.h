#ifndef CAR_H_
#define CAR_H_

#include "coords.h"

using namespace std;

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

void Vehicle::setPos(const Point &pos_cart, const Frenet &pos_frenet) {
  this->pos_cart = pos_cart;
  this->pos_frenet = pos_frenet;
}

Point Vehicle::getPos_cart() const {
  return pos_cart;
}

Frenet Vehicle::getPos_frenet() const {
  return pos_frenet;
}

void Vehicle::setVel_cart_m_per_s(const Point &vel) {
  this->vel_cart_m_per_s = vel;
}

Point Vehicle::getVel_cart_m_per_s() const {
  return vel_cart_m_per_s;
}

void Vehicle::setPos_cart(const Point &pos, const MapWaypoints &map_waypoints) {
  pos_cart = pos;
  pos_frenet = getFrenet(pos, 0, map_waypoints);
}

void Vehicle::setPos_frenet(const Frenet &pos,
                            const MapWaypoints &map_waypoints) {
  pos_frenet = pos;
  pos_cart = getXY(pos, map_waypoints);
}

void Vehicle::setVel_frenet_m_per_s(const Frenet &vel,
                                    const MapWaypoints &map_waypoints) {
  const Frenet &src = getPos_frenet();
  const Frenet &dst = Frenet { src.s + vel.s, src.d + vel.d };
  vel_cart_m_per_s = createCartVectorConnectingStartAndEnd(src, dst,
                                                           map_waypoints);
}

Frenet Vehicle::getVel_frenet_m_per_s(const MapWaypoints &map_waypoints) const {
  const Point &src = getPos_cart();
  const Point &dst = src + getVel_cart_m_per_s();
  return createFrenetVectorConnectingStartAndEnd(src, dst, map_waypoints);
}

#endif /* CAR_H_ */
