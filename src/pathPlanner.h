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
    map_waypoints.map_waypoints_x.push_back(x);
    map_waypoints.map_waypoints_y.push_back(y);
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

double distance(Point p1, Point p2) {
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  return sqrt(dx * dx + dy * dy);
}

int ClosestWaypoint(const Point &point, const MapWaypoints &map_waypoints) {

  double closestLen = 100000;  //large number
  int closestWaypoint = 0;

  for (int i = 0; i < map_waypoints.map_waypoints_x.size(); i++) {
    double map_x = map_waypoints.map_waypoints_x[i];
    double map_y = map_waypoints.map_waypoints_y[i];
    double dist = distance(point, Point { map_x, map_y });
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(const Point &point, double theta,
                 const MapWaypoints &map_waypoints) {

  int closestWaypoint = ClosestWaypoint(point, map_waypoints);

  double map_x = map_waypoints.map_waypoints_x[closestWaypoint];
  double map_y = map_waypoints.map_waypoints_y[closestWaypoint];

  double heading = atan2((map_y - point.y), (map_x - point.x));

  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == map_waypoints.map_waypoints_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
Frenet getFrenet(const Point &point, double theta,
                 const MapWaypoints &map_waypoints) {
  const vector<double> &maps_x = map_waypoints.map_waypoints_x;
  const vector<double> &maps_y = map_waypoints.map_waypoints_y;
  int next_wp = NextWaypoint(point, theta, map_waypoints);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = point.x - maps_x[prev_wp];
  double x_y = point.y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(Point { x_x, x_y }, Point { proj_x, proj_y });

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(Point { center_x, center_y },
                                Point { x_x, x_y });
  double centerToRef = distance(Point { center_x, center_y }, Point { proj_x,
                                    proj_y });

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(Point { maps_x[i], maps_y[i] }, Point { maps_x[i + 1],
                             maps_y[i + 1] });
  }

  frenet_s += distance(Point { 0, 0 }, Point { proj_x, proj_y });

  return Frenet { frenet_s, frenet_d };
}

// Transform from Frenet s,d coordinates to Cartesian x,y
Point getXY(double s, double d, const MapWaypoints &map_waypoints) {
  const vector<double> &maps_s = map_waypoints.map_waypoints_s;
  const vector<double> &maps_x = map_waypoints.map_waypoints_x;
  const vector<double> &maps_y = map_waypoints.map_waypoints_y;

  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]),
                         (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return Point { x, y };
}

void printInfo(const EgoCar &egoCar, const vector<Vehicle> &vehicles) {
  auto isCloserToEgoCar =
      [&egoCar](const Vehicle& vehicle1, const Vehicle& vehicle2) {
        double distance1 = distance(egoCar.pos_cart, vehicle1.pos_cart);
        double distance2 = distance(egoCar.pos_cart, vehicle2.pos_cart);
        return distance1 < distance2;
      };

  Vehicle vehicle = *min_element(vehicles.begin(), vehicles.end(),
                                 isCloserToEgoCar);

  cout << egoCar;
  cout << vehicle << endl;
}

bool isTooClose(const EgoCar& egoCar, const vector<Vehicle>& vehicles,
                const int prev_size, int lane) {
  for (int i = 0; i < vehicles.size(); i++) {
    float d = vehicles[i].pos_frenet.d;
    // if(d > 4*lane && d < 4*(lane + 1))
    if (d > 2 + 4 * lane - 2 && d < 2 + 4 * lane + 2) {
      double vx = vehicles[i].vx;
      double vy = vehicles[i].vy;
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = vehicles[i].pos_frenet.s;
      // TODO: replace magic number 0.02 with constant
      check_car_s += (double) prev_size * 0.02 * check_speed;
      // TODO: replace magic number 30 with constant
      if (check_car_s > egoCar.pos_frenet.s
          && check_car_s - egoCar.pos_frenet.s < 30) {
        // ref_vel = 29.5;
        return true;
      }
    }
  }

  return false;
}

double updateVelocity(bool too_close, double velocity) {
  if (too_close) {
    velocity -= .224;
  } else if (velocity < 49.5) {
    velocity += .224;
  }

  return velocity;
}

int updateLane(bool too_close, int lane) {
  if (too_close && lane > 0) {
    lane = 0;
  }

  return lane;
}

Points createPoints(const int prev_size, const EgoCar& egoCar,
                    ReferencePoint &refPoint, const PreviousData& previousData,
                    int& lane, const MapWaypoints &map_waypoints) {
  Points points;

  if (prev_size < 2) {
    double prev_car_x = egoCar.pos_cart.x - cos(egoCar.yaw);
    double prev_car_y = egoCar.pos_cart.y - sin(egoCar.yaw);
    points.xs.push_back(prev_car_x);
    points.xs.push_back(egoCar.pos_cart.x);
    points.ys.push_back(prev_car_y);
    points.ys.push_back(egoCar.pos_cart.y);
  } else {
    refPoint.point.x = previousData.previous_path_x[prev_size - 1];
    refPoint.point.y = previousData.previous_path_y[prev_size - 1];
    double ref_x_prev = previousData.previous_path_x[prev_size - 2];
    double ref_y_prev = previousData.previous_path_y[prev_size - 2];
    refPoint.yaw = atan2(refPoint.point.y - ref_y_prev,
                         refPoint.point.x - ref_x_prev);
    points.xs.push_back(ref_x_prev);
    points.xs.push_back(refPoint.point.x);
    points.ys.push_back(ref_y_prev);
    points.ys.push_back(refPoint.point.y);
  }
  // TODO: DRY: 2 + 4 * lane
  Point next_wp0 = getXY(egoCar.pos_frenet.s + 30, 2 + 4 * lane, map_waypoints);
  Point next_wp1 = getXY(egoCar.pos_frenet.s + 60, 2 + 4 * lane, map_waypoints);
  Point next_wp2 = getXY(egoCar.pos_frenet.s + 90, 2 + 4 * lane, map_waypoints);
  points.xs.push_back(next_wp0.x);
  points.xs.push_back(next_wp1.x);
  points.xs.push_back(next_wp2.x);
  points.ys.push_back(next_wp0.y);
  points.ys.push_back(next_wp1.y);
  points.ys.push_back(next_wp2.y);
  for (int i = 0; i < points.xs.size(); i++) {
    double shift_x = points.xs[i] - refPoint.point.x;
    double shift_y = points.ys[i] - refPoint.point.y;
    // TODO: reformulate as a matrix multiplication using Eigen
    points.xs[i] = shift_x * cos(-refPoint.yaw) - shift_y * sin(-refPoint.yaw);
    points.ys[i] = shift_x * sin(-refPoint.yaw) + shift_y * cos(-refPoint.yaw);
  }
  return points;
}

Points createNextVals(const Points &points, const int prev_size,
                      const PreviousData& previousData,
                      ReferencePoint &refPoint) {
  Points next_vals;

  tk::spline s;
  s.set_points(points.xs, points.ys);
  for (int i = 0; i < prev_size; i++) {
    next_vals.xs.push_back(previousData.previous_path_x[i]);
    next_vals.ys.push_back(previousData.previous_path_y[i]);
  }
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0;
  const int path_size = 50;
  for (int i = 1; i < path_size - prev_size; i++) {
    double N = target_dist / (0.02 * refPoint.vel / 2.24);
    double x_point = x_add_on + target_x / N;
    double y_point = s(x_point);
    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;
    // TODO: reformulate as a matrix multiplication using Eigen
    x_point = x_ref * cos(refPoint.yaw) - y_ref * sin(refPoint.yaw);
    y_point = x_ref * sin(refPoint.yaw) + y_ref * cos(refPoint.yaw);
    x_point += refPoint.point.x;
    y_point += refPoint.point.y;
    next_vals.xs.push_back(x_point);
    next_vals.ys.push_back(y_point);
  }

  return next_vals;
}

Points createPath(ReferencePoint &refPoint, int &lane,
                  const MapWaypoints &map_waypoints, EgoCar egoCar,
                  const PreviousData &previousData,
                  const vector<Vehicle> &vehicles) {

  // printInfo(egoCar, vehicles);

  const int prev_size = previousData.previous_path_x.size();

  if (prev_size > 0) {
    egoCar.pos_frenet.s = previousData.end_path.s;
  }

  bool too_close = isTooClose(egoCar, vehicles, prev_size, lane);
  lane = updateLane(too_close, lane);
  refPoint.vel = updateVelocity(too_close, refPoint.vel);

  // TODO: define and use operator= instead
  refPoint.point = egoCar.pos_cart;
  refPoint.yaw = deg2rad(egoCar.yaw);

  Points points = createPoints(prev_size, egoCar, refPoint, previousData, lane,
                               map_waypoints);

  return createNextVals(points, prev_size, previousData, refPoint);
}

EgoCar createEgoCar(
    const nlohmann::basic_json<std::map, std::vector,
        std::__cxx11::basic_string<char, std::char_traits<char>,
            std::allocator<char> >, bool, long, unsigned long, double,
        std::allocator, nlohmann::adl_serializer> &j) {
  EgoCar egoCar;
  egoCar.pos_cart = Point { j[1]["x"], j[1]["y"] };
  egoCar.pos_frenet = Frenet { j[1]["s"], j[1]["d"] };
  egoCar.yaw = j[1]["yaw"];
  egoCar.speed = j[1]["speed"];
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
  previousData.previous_path_x = previous_path_x;
  vector<double> previous_path_y = j[1]["previous_path_y"];
  previousData.previous_path_y = previous_path_y;

  // Previous path's end s and d values
  previousData.end_path.s = j[1]["end_path_s"];
  previousData.end_path.d = j[1]["end_path_d"];
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
    vehicle.pos_cart.x = sensor_fusion[i][X];
    vehicle.pos_cart.y = sensor_fusion[i][Y];
    vehicle.vx = sensor_fusion[i][VX];
    vehicle.vy = sensor_fusion[i][VY];
    vehicle.pos_frenet.s = sensor_fusion[i][S];
    vehicle.pos_frenet.d = sensor_fusion[i][D];
    vehicles.push_back(vehicle);
  }
  return vehicles;
}

#endif /* PATHPLANNER_H_ */
