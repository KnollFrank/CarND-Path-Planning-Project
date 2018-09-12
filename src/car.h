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

#endif /* CAR_H_ */
