#ifndef MAIN_H_
#define MAIN_H_

using namespace std;

struct MapWaypoints {
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
};

struct EgoCar {
  // TODO: remove car_ prefix
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);
};

ostream& operator<<(ostream& os, const EgoCar& egoCar)
{
    os << "EgoCar:" << endl;
    os << "  (x, y) = (" << egoCar.car_x << ", " << egoCar.car_y << ")" << endl;
    os << "  (s, d) = (" << egoCar.car_s << ", " << egoCar.car_d << ")" << endl;
    os << "  yaw = " << egoCar.car_yaw << endl;
    os << "  speed = " << egoCar.car_speed << endl;
    return os;
}

struct PreviousData {
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
};

struct Vehicle {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;

  friend ostream& operator<<(ostream& os, const Vehicle& vehicle);
};

ostream& operator<<(ostream& os, const Vehicle& vehicle)
{
    os << "Vehicle(" << vehicle.id << "):" << endl;
    os << "  (x, y) = (" << vehicle.x << ", " << vehicle.y << ")" << endl;
    os << "  (vx, vy) = (" << vehicle.vx << ", " << vehicle.vy << ")" << endl;
    os << "  (s, d) = (" << vehicle.s << ", " << vehicle.d << ")" << endl;
    return os;
}

#endif /* MAIN_H_ */
