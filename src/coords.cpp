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

