#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <tuple>
#include "main.h"
#include "spline.h"
#include "car.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}
double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}

double mph2meter_per_sec(double t) {
  return t / 2.24;
}

double meter_per_sec2mph(double t) {
  return t * 2.24;
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

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int ClosestWaypoint(const Point &point, const MapWaypoints &map_waypoints) {

  double closestLen = 100000;  //large number
  int closestWaypoint = 0;

  for (int i = 0; i < map_waypoints.map_waypoints.size(); i++) {
    double dist = point.distanceTo(map_waypoints.map_waypoints[i]);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(const Point &point, double theta_rad,
                 const MapWaypoints &map_waypoints) {

  int closestWaypoint = ClosestWaypoint(point, map_waypoints);
  Point map = map_waypoints.map_waypoints[closestWaypoint];
  double heading = (map - point).getHeading();
  double angle = fabs(theta_rad - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == map_waypoints.map_waypoints.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet getFrenet(const Point &point, double theta_rad,
                 const MapWaypoints &map_waypoints) {
  const vector<Point> &maps = map_waypoints.map_waypoints;
  int next_wp = NextWaypoint(point, theta_rad, map_waypoints);

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

// Transform from Frenet s,d coordinates to Cartesian x,y
Point getXY(const Frenet &pos, const MapWaypoints &map_waypoints) {
  const vector<double> &maps_s = map_waypoints.map_waypoints_s;
  const vector<Point> &maps = map_waypoints.map_waypoints;

  int prev_wp = -1;

  while (pos.s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps.size();

  double heading = (maps[wp2] - maps[prev_wp]).getHeading();
  // the x,y,s along the segment
  double seg_s = pos.s - maps_s[prev_wp];

  Point seg = maps[prev_wp] + Point::fromAngle(heading) * seg_s;
  double perp_heading = heading - pi() / 2;

  return seg + Point::fromAngle(perp_heading) * pos.d;
}

Point createCartVectorConnectingStartAndEnd(const Frenet &start,
                                            const Frenet &end,
                                            const MapWaypoints &map_waypoints) {
  return getXY(end, map_waypoints) - getXY(start, map_waypoints);
}

Frenet createFrenetVectorConnectingStartAndEnd(
    const Point &start, const Point &end, const MapWaypoints &map_waypoints) {

  return getFrenet(end, 0, map_waypoints) - getFrenet(start, 0, map_waypoints);
}

void printInfo(const EgoCar &egoCar, const vector<Vehicle> &vehicles) {
  auto isCloserToEgoCar =
      [&egoCar](const Vehicle& vehicle1, const Vehicle& vehicle2) {
        double distance1 = egoCar.getPos_cart().distanceTo(vehicle1.getPos_cart());
        double distance2 = egoCar.getPos_cart().distanceTo(vehicle2.getPos_cart());
        return distance1 < distance2;
      };

  Vehicle vehicle = *min_element(vehicles.begin(), vehicles.end(),
                                 isCloserToEgoCar);

  cout << egoCar;
  cout << vehicle << endl;
}

double sizeOfLane() {
  return 4;
}

double startOfLane(int lane) {
  return sizeOfLane() * lane;
}

double getMiddleOfLane(Lane lane) {
  return startOfLane(lane) + sizeOfLane() / 2;
}

double endOfLane(Lane lane) {
  return startOfLane(lane + 1);
}

bool isInLane(float d, Lane lane) {
  return startOfLane(lane) < d && d < endOfLane(lane);
}

bool isVehicleInLane(const Vehicle &vehicle, Lane lane) {
  return isInLane(vehicle.getPos_frenet().d, lane);
}

bool willVehicleBeWithin30MetersAheadOfEgoCar(const EgoCar& egoCar,
                                              const Vehicle &vehicle,
                                              const int prev_size, double dt) {
  double check_speed = vehicle.getVel_cart_m_per_s().len();
  double check_vehicle_s = vehicle.getPos_frenet().s
      + prev_size * dt * check_speed;
  // TODO: replace magic number 30 with constant
  return check_vehicle_s > egoCar.getPos_frenet().s
      && check_vehicle_s - egoCar.getPos_frenet().s < 30;
}

bool isEgoCarTooCloseToAnyVehicleInLane(const EgoCar& egoCar,
                                        const vector<Vehicle>& vehicles,
                                        const int prev_size, Lane lane,
                                        double dt) {
  auto isEgoCarTooCloseToVehicleInLane =
      [&egoCar, prev_size, lane, dt]
      (const Vehicle &vehicle) {
        return isVehicleInLane(vehicle, lane) && willVehicleBeWithin30MetersAheadOfEgoCar(egoCar, vehicle, prev_size, dt);};

  return std::any_of(vehicles.cbegin(), vehicles.cend(),
                     isEgoCarTooCloseToVehicleInLane);
}

double getNewVelocity(bool too_close, double vel_mph) {
  if (too_close || vel_mph > 50) {
    vel_mph -= .224;
  } else if (vel_mph < 49.5) {
    vel_mph += .224;
  }

  return vel_mph;
}

Lane getNewLane(bool too_close, Lane lane) {
  if (too_close && lane > Lane::LEFT) {
    lane = Lane::LEFT;
  }

  return lane;
}

tuple<Point, Point> createRotatedVectors(double angle_rad) {
  Point e1 = Point { cos(angle_rad), sin(angle_rad) };
  Point e2 = Point { -sin(angle_rad), cos(angle_rad) };
  return make_tuple(e1, e2);
}

Point transform(const tuple<Point, Point> &rotatedVectors, const Point& point) {
  return std::get < 0 > (rotatedVectors) * point.x + std::get < 1
      > (rotatedVectors) * point.y;
}

Path createPoints(const int prev_size, const EgoCar& egoCar,
                  ReferencePoint &refPoint, const PreviousData& previousData,
                  Lane lane, const MapWaypoints &map_waypoints) {
  Path path;

  if (prev_size < 2) {
    Point prev = egoCar.getPos_cart()
        - Point::fromAngle(deg2rad(egoCar.yaw_deg));
    path.points.push_back(prev);
    path.points.push_back(egoCar.getPos_cart());
  } else {
    refPoint.point = previousData.previous_path.points[prev_size - 1];
    Point prev = previousData.previous_path.points[prev_size - 2];
    refPoint.yaw_rad = (refPoint.point - prev).getHeading();
    path.points.push_back(prev);
    path.points.push_back(refPoint.point);
  }
  Point next_wp0 = getXY(Frenet { egoCar.getPos_frenet().s + 30,
                             getMiddleOfLane(lane) },
                         map_waypoints);
  Point next_wp1 = getXY(Frenet { egoCar.getPos_frenet().s + 60,
                             getMiddleOfLane(lane) },
                         map_waypoints);
  Point next_wp2 = getXY(Frenet { egoCar.getPos_frenet().s + 90,
                             getMiddleOfLane(lane) },
                         map_waypoints);

  path.points.push_back(next_wp0);
  path.points.push_back(next_wp1);
  path.points.push_back(next_wp2);

  tuple<Point, Point> rotatedVectors = createRotatedVectors(-refPoint.yaw_rad);
  for (int i = 0; i < path.points.size(); i++) {
    path.points[i] = transform(rotatedVectors, path.points[i] - refPoint.point);
  }

  // TODO: extract method, sort_and_remove_duplicates
  std::sort(path.points.begin(), path.points.end(),
            [](const Point &p1, const Point &p2) {return p1.x < p2.x;});
  path.points.erase(
      unique(path.points.begin(), path.points.end(),
             [](const Point &p1, const Point &p2) {return p1.x == p2.x;}),
      path.points.end());

  return path;
}

tuple<vector<double>, vector<double>> getPoints(const Path &path) {
  vector<double> xs;
  vector<double> ys;
  for (const Point &point : path.points) {
    xs.push_back(point.x);
    ys.push_back(point.y);
  }

  return make_tuple(xs, ys);
}

Point createSplinePoint(double x, const tk::spline& s) {
  return Point { x, s(x) };
}

Path createNextVals(const Path &path, const int prev_size,
                    const PreviousData& previousData,
                    const ReferencePoint &refPoint, double dt) {
  Path next_vals;

  vector<double> xs;
  vector<double> ys;
  tie(xs, ys) = getPoints(path);

  tk::spline s;
  s.set_points(xs, ys);
  for (int i = 0; i < prev_size; i++) {
    next_vals.points.push_back(previousData.previous_path.points[i]);
  }
  Point target = createSplinePoint(30.0, s);
  double x_add_on = 0;
  const int path_size = 50;
  tuple<Point, Point> rotatedVectors = createRotatedVectors(refPoint.yaw_rad);
  double N = target.len() / (dt * mph2meter_per_sec(refPoint.vel_mph));
  for (int i = 1; i < path_size - prev_size; i++) {
    Point point = createSplinePoint(x_add_on + target.x / N, s);
    x_add_on = point.x;
    next_vals.points.push_back(
        transform(rotatedVectors, point) + refPoint.point);
  }

  return next_vals;
}

Path createPath(ReferencePoint &refPoint, Lane &lane,
                const MapWaypoints &map_waypoints, EgoCar egoCar,
                const PreviousData &previousData,
                const vector<Vehicle> &vehicles, double dt) {

  // printInfo(egoCar, vehicles);

  const int prev_size = previousData.previous_path.points.size();

  if (prev_size > 0) {
    egoCar.setPos_frenet(Frenet { previousData.end_path.s,
                             egoCar.getPos_frenet().d },
                         map_waypoints);
  }

  bool too_close = isEgoCarTooCloseToAnyVehicleInLane(egoCar, vehicles,
                                                      prev_size, lane, dt);
  lane = getNewLane(too_close, lane);
  refPoint.vel_mph = getNewVelocity(too_close, refPoint.vel_mph);
  refPoint.point = egoCar.getPos_cart();
  refPoint.yaw_rad = deg2rad(egoCar.yaw_deg);

  Path path = createPoints(prev_size, egoCar, refPoint, previousData, lane,
                           map_waypoints);

  return createNextVals(path, prev_size, previousData, refPoint, dt);
}

EgoCar createEgoCar(
    const nlohmann::basic_json<std::map, std::vector,
        std::__cxx11::basic_string<char, std::char_traits<char>,
            std::allocator<char> >, bool, long, unsigned long, double,
        std::allocator, nlohmann::adl_serializer> &j) {
  EgoCar egoCar;
  egoCar.setPos(Point { j[1]["x"], j[1]["y"] },
                Frenet { j[1]["s"], j[1]["d"] });
  egoCar.yaw_deg = j[1]["yaw"];
  egoCar.speed_mph = j[1]["speed"];
  return egoCar;
}

PreviousData createPreviousData(
    const nlohmann::basic_json<std::map, std::vector,
        std::__cxx11::basic_string<char, std::char_traits<char>,
            std::allocator<char> >, bool, long, unsigned long, double,
        std::allocator, nlohmann::adl_serializer> &j) {
  PreviousData previousData;
  // Previous path data given to the Planner
  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector<double> previous_path_y = j[1]["previous_path_y"];
  for (int i = 0; i < previous_path_x.size(); i++) {
    previousData.previous_path.points.push_back(Point { previous_path_x[i],
        previous_path_y[i] });
  }

  // Previous path's end s and d values
  previousData.end_path = Frenet { j[1]["end_path_s"], j[1]["end_path_d"] };
  return previousData;
}

// Sensor Fusion Data, a list of all other cars on the same side of the road.
vector<Vehicle> createVehicles(
    const nlohmann::basic_json<std::map, std::vector,
        std::__cxx11::basic_string<char, std::char_traits<char>,
            std::allocator<char> >, bool, long, unsigned long, double,
        std::allocator, nlohmann::adl_serializer> &sensor_fusion) {
  enum sensor_fusion_index {
    ID = 0,
    X = 1,
    Y = 2,
    VX = 3,
    VY = 4,
    S = 5,
    D = 6
  };

  vector<Vehicle> vehicles;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    Vehicle vehicle;
    vehicle.id = sensor_fusion[i][ID];
    vehicle.setPos(Point { sensor_fusion[i][X], sensor_fusion[i][Y] }, Frenet {
                       sensor_fusion[i][S], sensor_fusion[i][D] });
    vehicle.setVel_cart_m_per_s(Point { sensor_fusion[i][VX],
        sensor_fusion[i][VY] });
    vehicles.push_back(vehicle);
  }
  return vehicles;
}

#endif /* PATHPLANNER_H_ */
