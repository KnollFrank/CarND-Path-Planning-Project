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
#include "spline.h"
#include "car.h"
#include "mathfuns.h"
#include "lane.h"
#include "path.h"
#include "previousData.h"
#include "coords/coordinateSystem.h"

using namespace std;

// for convenience
using json = nlohmann::json;

struct ReferencePoint {
  Point point;
  double yaw_rad;
  double vel_mph;
};

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

Lane getNewLane(bool too_close, Lane lane) {
  if (too_close && lane > Lane::LEFT) {
    lane = Lane::LEFT;
  }

  return lane;
}

CoordinateSystem createRotatedVectors(const Point& origin, double angle_rad) {
  Point e1 = Point { cos(angle_rad), sin(angle_rad) };
  Point e2 = Point { -sin(angle_rad), cos(angle_rad) };
  return CoordinateSystem { origin, e1, e2 };
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

class PathPlanner {

 public:
  PathPlanner(const CoordsConverter& coordsConverter, ReferencePoint& refPoint,
              Lane& lane);

  Path createPath(EgoCar egoCar, const PreviousData &previousData,
                  const vector<Vehicle> &vehicles, double dt);

 private:
  bool isEgoCarTooCloseToAnyVehicleInLane(const EgoCar& egoCar,
                                          const vector<Vehicle>& vehicles,
                                          const int prev_size, double dt);
  bool willVehicleBeWithin30MetersAheadOfEgoCar(const EgoCar& egoCar,
                                                const Vehicle &vehicle,
                                                const int prev_size, double dt);
  double getNewVelocity(bool too_close, double vel_mph);
  Path createPoints(const int prev_size, const EgoCar& egoCar,
                    const PreviousData& previousData);
  Path createNextVals(const Path &path, const int prev_size,
                      const PreviousData& previousData, double dt);

  const CoordsConverter& coordsConverter;
  ReferencePoint& refPoint;
  Lane& lane;
};

PathPlanner::PathPlanner(const CoordsConverter& _coordsConverter,
                         ReferencePoint& _refPoint, Lane& _lane)
    : coordsConverter(_coordsConverter),
      refPoint(_refPoint),
      lane(_lane) {
}

Path PathPlanner::createPath(EgoCar egoCar, const PreviousData &previousData,
                             const vector<Vehicle> &vehicles, double dt) {

  // printInfo(egoCar, vehicles);

  const int prev_size = previousData.previous_path.points.size();

  if (prev_size > 0) {
    egoCar.setPos_frenet(Frenet { previousData.end_path.s,
        egoCar.getPos_frenet().d });
  }

  bool too_close = isEgoCarTooCloseToAnyVehicleInLane(egoCar, vehicles,
                                                      prev_size, dt);
  lane = getNewLane(too_close, lane);
  refPoint.vel_mph = getNewVelocity(too_close, refPoint.vel_mph);
  refPoint.point = egoCar.getPos_cart();
  refPoint.yaw_rad = deg2rad(egoCar.yaw_deg);

  Path path = createPoints(prev_size, egoCar, previousData);

  return createNextVals(path, prev_size, previousData, dt);
}

bool PathPlanner::isEgoCarTooCloseToAnyVehicleInLane(
    const EgoCar& egoCar, const vector<Vehicle>& vehicles, const int prev_size,
    double dt) {
  auto isEgoCarTooCloseToVehicleInLane =
      [&]
      (const Vehicle &vehicle) {
        return isVehicleInLane(vehicle, lane) && willVehicleBeWithin30MetersAheadOfEgoCar(egoCar, vehicle, prev_size, dt);};

  return std::any_of(vehicles.cbegin(), vehicles.cend(),
                     isEgoCarTooCloseToVehicleInLane);
}

bool PathPlanner::willVehicleBeWithin30MetersAheadOfEgoCar(
    const EgoCar& egoCar, const Vehicle &vehicle, const int prev_size,
    double dt) {
  double check_speed = vehicle.getVel_cart_m_per_s().len();
  double check_vehicle_s = vehicle.getPos_frenet().s
      + prev_size * dt * check_speed;
  // TODO: replace magic number 30 with constant
  return check_vehicle_s > egoCar.getPos_frenet().s
      && check_vehicle_s - egoCar.getPos_frenet().s < 30;
}

double PathPlanner::getNewVelocity(bool too_close, double vel_mph) {
  if (too_close || vel_mph > 50) {
    vel_mph -= .224;
  } else if (vel_mph < 49.5) {
    vel_mph += .224;
  }

  return vel_mph;
}

Path PathPlanner::createPoints(const int prev_size, const EgoCar& egoCar,
                               const PreviousData& previousData) {
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
  Point next_wp0 = coordsConverter.getXY(Frenet { egoCar.getPos_frenet().s + 30,
      getMiddleOfLane(lane) });
  Point next_wp1 = coordsConverter.getXY(Frenet { egoCar.getPos_frenet().s + 60,
      getMiddleOfLane(lane) });
  Point next_wp2 = coordsConverter.getXY(Frenet { egoCar.getPos_frenet().s + 90,
      getMiddleOfLane(lane) });

  path.points.push_back(next_wp0);
  path.points.push_back(next_wp1);
  path.points.push_back(next_wp2);

  CoordinateSystem coordinateSystem = createRotatedVectors(Point { 0, 0 },
                                                           -refPoint.yaw_rad);
  for (int i = 0; i < path.points.size(); i++) {
    Point point = path.points[i] - refPoint.point;
    path.points[i] = coordinateSystem.transform(point.x, point.y);
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

Path PathPlanner::createNextVals(const Path &path, const int prev_size,
                                 const PreviousData& previousData, double dt) {
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
  CoordinateSystem coordinateSystem = createRotatedVectors(refPoint.point,
                                                           refPoint.yaw_rad);
  double N = target.len() / (dt * mph2meter_per_sec(refPoint.vel_mph));
  for (int i = 1; i < path_size - prev_size; i++) {
    Point point = createSplinePoint(x_add_on + target.x / N, s);
    x_add_on = point.x;
    next_vals.points.push_back(coordinateSystem.transform(point.x, point.y));
  }

  return next_vals;
}

#endif /* PATHPLANNER_H_ */
