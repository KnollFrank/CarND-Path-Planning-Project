#include "coords.h"
#include "main.h"
#include "pathPlanner.h"

Point Point::fromAngle(double angle_rad) {
  return Point { cos(angle_rad), sin(angle_rad) };
}

double Point::len() const {
  return distanceTo(Point { 0, 0 });
}

double Point::scalarProd(const Point &point) const {
  return x * point.x + y * point.y;
}

double Point::distanceTo(const Point &point) const {
  Point diff = point - *this;
  return sqrt(diff.scalarProd(diff));
}

double Point::getHeading() const {
  return atan2(y, x);
}

Point Point::operator+(const Point &other) const {
  return Point { x + other.x, y + other.y };
}

Point Point::operator-(const Point &other) const {
  return *this + (other * -1);
}

Point Point::operator*(double scalar) const {
  return Point { x * scalar, y * scalar };
}

Point& Point::operator=(const Point &point) {
  // self-assignment guard
  if (this == &point)
    return *this;

  // do the copy
  x = point.x;
  y = point.y;

  // return the existing object so we can chain this operator
  return *this;
}

Frenet Frenet::operator+(const Frenet &other) const {
  return Frenet { s + other.s, d + other.d };
}

Frenet Frenet::operator-(const Frenet &other) const {
  return *this + (other * -1);
}

Frenet Frenet::operator*(double scalar) const {
  return Frenet { s * scalar, d * scalar };
}

ostream& operator<<(ostream& os, const Point& point) {
  os << "Point(x = " << point.x << ", y = " << point.y << ")";
  return os;
}

ostream& operator<<(ostream& os, const Frenet& frenet) {
  os << "Frenet(s = " << frenet.s << ", d = " << frenet.d << ")";
  return os;
}

bool operator==(const Frenet& lhs, const Frenet& rhs) {
  return lhs.s == rhs.s && lhs.d == rhs.d;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet getFrenet2(const Point& point, const MapWaypoints& map_waypoints) {
  const vector<Point> &maps = map_waypoints.map_waypoints;
  int next_wp = NextWaypoint(point, 0, map_waypoints);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps.size() - 1;
  }

  const Point n = maps[next_wp] - maps[prev_wp];
  const Point x = point - maps[prev_wp];

  // find the projection of x onto n
  // TODO: warum nicht /n.len() ?
  double proj_norm = x.scalarProd(n) / n.scalarProd(n);
  const Point proj = n * proj_norm;
  double frenet_d = x.distanceTo(proj);

  //see if d value is positive or negative by comparing it to a center point

  const Point center = Point { 1000, 2000 } - maps[prev_wp];
  double centerToPos = center.distanceTo(x);
  double centerToRef = center.distanceTo(proj);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += maps[i].distanceTo(maps[i + 1]);
  }

  frenet_s += proj.len();

  return Frenet { frenet_s, frenet_d };
}

// Load up map values for waypoint's x,y,s and d normalized normal vectors
MapWaypoints read_map_waypoints() {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  MapWaypoints map_waypoints;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints.map_waypoints.push_back(Point { x, y });
    map_waypoints.map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  return map_waypoints;
}

