#ifndef MAIN_H_
#define MAIN_H_

using namespace std;

struct MapWaypoints {
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
};

struct Point {
  double x;
  double y;

  friend ostream& operator<<(ostream& os, const Point& point);
};

ostream& operator<<(ostream& os, const Point& point)
{
    os << "Point(x = " << point.x << ", y = " << point.y << ")";
    return os;
}

struct Frenet {
  double s;
  double d;

  friend ostream& operator<<(ostream& os, const Frenet& frenet);
};

ostream& operator<<(ostream& os, const Frenet& frenet)
{
    os << "Frenet(s = " << frenet.s << ", d = " << frenet.d << ")";
    return os;
}

struct EgoCar {
  Point pos_cart;
  Frenet pos_frenet;
  double yaw;
  double speed;

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);
};

ostream& operator<<(ostream& os, const EgoCar& egoCar)
{
    os << "EgoCar:" << endl;
    os << "  pos_cart = " << egoCar.pos_cart << endl;
    os << "  pos_frenet = " << egoCar.pos_frenet << endl;
    os << "  yaw = " << egoCar.yaw << endl;
    os << "  speed = " << egoCar.speed << endl;
    return os;
}

struct PreviousData {
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  Frenet end_path;
};

struct Vehicle {
  int id;
  Point pos_cart;
  double vx;
  double vy;
  Frenet pos_frenet;

  friend ostream& operator<<(ostream& os, const Vehicle& vehicle);
};

ostream& operator<<(ostream& os, const Vehicle& vehicle)
{
    os << "Vehicle(" << vehicle.id << "):" << endl;
    os << "  pos_cart = " << vehicle.pos_cart << endl;
    os << "  (vx, vy) = (" << vehicle.vx << ", " << vehicle.vy << ")" << endl;
    os << "  pos_frenet = " << vehicle.pos_frenet << endl;
    return os;
}

struct Points {
  vector<double> xs;
  vector<double> ys;
};

struct ReferencePoint {
  Point point;
  double yaw;
  double vel;
};

#endif /* MAIN_H_ */
