#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include "json.hpp"
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <vector>

#include "car.h"
#include "coords/cart.h"
#include "coords/coordinateSystem.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "funs.h"
#include "lane.h"
#include "path.h"
#include "previousData.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

struct ReferencePoint {
  Frenet point;
  double yaw_rad;
  double vel_mph;
};

void printInfo(const EgoCar& egoCar, const vector<Vehicle>& vehicles) {
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

class PathPlanner {

 public:
  PathPlanner(const CoordsConverter& coordsConverter, ReferencePoint& refPoint,
              Lane& lane, double dt);

  Path createPath(EgoCar egoCar, const PreviousData& previousData,
                  const vector<Vehicle>& vehicles);

 private:
  bool isEgoCarTooCloseToAnyVehicleInLane(const EgoCar& egoCar,
                                          const vector<Vehicle>& vehicles,
                                          const int prev_size);
  bool willVehicleBeWithin30MetersAheadOfEgoCar(const EgoCar& egoCar,
                                                const Vehicle& vehicle,
                                                const int prev_size);
  double getNewVelocity(bool too_close, double vel_mph);
  Lane getNewLane(bool too_close, Lane lane);
  CoordinateSystem createRotatedCoordinateSystem(const Frenet& origin,
                                                 double angle_rad);
  FrenetCart createSplinePoint(double x, const tk::spline& spline);
  void sort_and_remove_duplicates(vector<FrenetCart>& points);
  std::vector<FrenetCart> createPointsFromPreviousData(
      const EgoCar& egoCar, const PreviousData& previousData);
  std::vector<FrenetCart> createNewPoints(const EgoCar& egoCar);
  vector<FrenetCart> workWithPathInCarsCoordinateSystem(
      const Path& path,
      const function<vector<FrenetCart>(const Path& carsPath)>& transformCarsPath2Points);
  vector<FrenetCart> enterCarsCoordinateSystem(
      const Frenet& origin, const double angle_rad,
      const vector<FrenetCart>& points);
  vector<FrenetCart> leaveCarsCoordinateSystem(
      const Frenet& origin, double angle_rad, const vector<FrenetCart>& points);
  std::vector<FrenetCart> createSplinePoints(const tk::spline& spline,
                                             const int num);
  vector<FrenetCart> transform(const CoordinateSystem& coordinateSystem,
                               const vector<FrenetCart>& points) const;
  std::vector<double> createSVals(const tk::spline& spline, const int num);
  void addPointsFromPreviousData(Path& path, const EgoCar& egoCar,
                                 const PreviousData& previousData);
  void addNewPoints(Path& path, const EgoCar& egoCar);

  const CoordsConverter& coordsConverter;
  // TODO: refPoint und lane sollen unveränderbare Rückgabewerte von createPath sein.
  ReferencePoint& refPoint;
  Lane& lane;
  double dt;
  const int path_size = 50;
};

PathPlanner::PathPlanner(const CoordsConverter& _coordsConverter,
                         ReferencePoint& _refPoint, Lane& _lane, double _dt)
    : coordsConverter(_coordsConverter),
      refPoint(_refPoint),
      lane(_lane),
      dt(_dt) {
}

vector<FrenetCart> PathPlanner::enterCarsCoordinateSystem(
    const Frenet& origin, const double angle_rad,
    const vector<FrenetCart>& points) {

  CoordinateSystem coordinateSystem = createRotatedCoordinateSystem(
      Frenet::zero(), angle_rad);
  vector<FrenetCart> origin2points = map2<FrenetCart, FrenetCart>(
      points, [&](const FrenetCart& point) {
        return FrenetCart(point.getFrenet(coordsConverter) - origin);
      });
  return transform(coordinateSystem, origin2points);
}

vector<FrenetCart> PathPlanner::leaveCarsCoordinateSystem(
    const Frenet& origin, double angle_rad, const vector<FrenetCart>& points) {

  return transform(createRotatedCoordinateSystem(origin, angle_rad), points);
}

vector<FrenetCart> PathPlanner::workWithPathInCarsCoordinateSystem(
    const Path& path,
    const function<vector<FrenetCart>(const Path& carsPath)>& transformCarsPath2Points) {

  Path carsPath;
  carsPath.points = enterCarsCoordinateSystem(refPoint.point, -refPoint.yaw_rad,
                                              path.points);
  sort_and_remove_duplicates(carsPath.points);
  vector<FrenetCart> points = transformCarsPath2Points(carsPath);
  return leaveCarsCoordinateSystem(refPoint.point, refPoint.yaw_rad, points);
}

Path PathPlanner::createPath(EgoCar egoCar, const PreviousData& previousData,
                             const vector<Vehicle>& vehicles) {

  // printInfo(egoCar, vehicles);

  if (previousData.sizeOfPreviousPath() > 0) {
    egoCar.setPos_frenet(previousData.end_path);
  }

  bool too_close = isEgoCarTooCloseToAnyVehicleInLane(
      egoCar, vehicles, previousData.sizeOfPreviousPath());
  lane = getNewLane(too_close, lane);
  refPoint.vel_mph = getNewVelocity(too_close, refPoint.vel_mph);
  refPoint.point = egoCar.getPos_frenet();
  refPoint.yaw_rad = deg2rad(egoCar.yaw_deg);

//  Path path;
//  addPointsFromPreviousData(path, egoCar, previousData);
//  addNewPoints(path, egoCar);
//
//  vector<FrenetCart> points =
//      workWithPathInCarsCoordinateSystem(
//          path,
//          [&](const Path& carsPath) {
//            return createSplinePoints(carsPath.asSpline(coordsConverter), path_size - previousData.sizeOfPreviousPath());
//          });
//
//  Path next_vals;
//  appendSnd2Fst(next_vals.points, previousData.previous_path.points);
//  appendSnd2Fst(next_vals.points, points);

  Path next_vals;
  double dist_inc = 0.5;
  for (int i = 0; i < 50; i++) {
    double next_s = egoCar.getPos_frenet().s + dist_inc * (i + 1);
    double next_d = 6;
    next_vals.points.push_back(FrenetCart(Frenet { next_s, next_d }));
  }

  return next_vals;
}

bool PathPlanner::isEgoCarTooCloseToAnyVehicleInLane(
    const EgoCar& egoCar, const vector<Vehicle>& vehicles,
    const int prev_size) {
  auto isEgoCarTooCloseToVehicleInLane =
      [&]
      (const Vehicle& vehicle) {
        return isVehicleInLane(vehicle, lane) && willVehicleBeWithin30MetersAheadOfEgoCar(egoCar, vehicle, prev_size);};

  return std::any_of(vehicles.cbegin(), vehicles.cend(),
                     isEgoCarTooCloseToVehicleInLane);
}

bool PathPlanner::willVehicleBeWithin30MetersAheadOfEgoCar(
    const EgoCar& egoCar, const Vehicle& vehicle, const int prev_size) {
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

Lane PathPlanner::getNewLane(bool too_close, Lane lane) {
  if (too_close && lane > Lane::LEFT) {
    lane = Lane::LEFT;
  }

  return lane;
}

vector<FrenetCart> PathPlanner::createPointsFromPreviousData(
    const EgoCar& egoCar, const PreviousData& previousData) {

  vector<FrenetCart> points;
  if (previousData.sizeOfPreviousPath() < 2) {
    Frenet prev = egoCar.getPos_frenet()
        - Frenet::fromAngle(deg2rad(egoCar.yaw_deg));
    points.push_back(FrenetCart(prev));
    points.push_back(FrenetCart(egoCar.getPos_frenet()));
  } else {
    refPoint.point = previousData.previous_path.points[previousData
        .sizeOfPreviousPath() - 1].getFrenet(coordsConverter);
    Frenet prev = previousData.previous_path.points[previousData
        .sizeOfPreviousPath() - 2].getFrenet(coordsConverter);
    refPoint.yaw_rad = (refPoint.point - prev).getHeading();
    points.push_back(FrenetCart(prev));
    points.push_back(FrenetCart(refPoint.point));
  }
  return points;
}

vector<FrenetCart> PathPlanner::createNewPoints(const EgoCar& egoCar) {
  vector<FrenetCart> points;
  points.push_back(FrenetCart(Frenet { egoCar.getPos_frenet().s + 30,
      getMiddleOfLane(lane) }));
  points.push_back(FrenetCart(Frenet { egoCar.getPos_frenet().s + 60,
      getMiddleOfLane(lane) }));
  points.push_back(FrenetCart(Frenet { egoCar.getPos_frenet().s + 90,
      getMiddleOfLane(lane) }));
  return points;
}

void PathPlanner::addPointsFromPreviousData(Path& path, const EgoCar& egoCar,
                                            const PreviousData& previousData) {
  appendSnd2Fst(path.points,
                createPointsFromPreviousData(egoCar, previousData));
}

void PathPlanner::addNewPoints(Path& path, const EgoCar& egoCar) {
  appendSnd2Fst(path.points, createNewPoints(egoCar));
}

void PathPlanner::sort_and_remove_duplicates(vector<FrenetCart>& points) {
  std::sort(
      points.begin(),
      points.end(),
      [&](const FrenetCart& p1, const FrenetCart& p2) {return p1.getFrenet(coordsConverter).s < p2.getFrenet(coordsConverter).s;});
  points.erase(
      unique(
          points.begin(),
          points.end(),
          [&](const FrenetCart& p1, const FrenetCart& p2) {return p1.getFrenet(coordsConverter).s == p2.getFrenet(coordsConverter).s;}),
      points.end());
}

vector<FrenetCart> PathPlanner::transform(
    const CoordinateSystem& coordinateSystem,
    const vector<FrenetCart>& points) const {

  return map2<FrenetCart, FrenetCart>(
      points,
      [&](const FrenetCart& point) {
        return FrenetCart(coordinateSystem.transform(point.getFrenet(coordsConverter)));});
}

vector<double> PathPlanner::createSVals(const tk::spline& spline,
                                        const int num) {
  vector<double> s_vals;
  Frenet target = createSplinePoint(30.0, spline).getFrenet(coordsConverter);
  double s_add_on = 0;
  double N = target.len() / (dt * mph2meter_per_sec(refPoint.vel_mph));
  for (int i = 0; i < num; i++) {
    s_add_on += target.s / N;
    s_vals.push_back(s_add_on);
  }
  return s_vals;
}

vector<FrenetCart> PathPlanner::createSplinePoints(const tk::spline& spline,
                                                   const int num) {

  vector<double> s_vals = createSVals(spline, num);
  vector<FrenetCart> points = map2<double, FrenetCart>(
      s_vals,
      [&](const double s_val) {return createSplinePoint(s_val, spline);});
  return points;
}

CoordinateSystem PathPlanner::createRotatedCoordinateSystem(
    const Frenet& origin, double angle_rad) {
  Frenet e1 = Frenet { cos(angle_rad), sin(angle_rad) };
  Frenet e2 = Frenet { -sin(angle_rad), cos(angle_rad) };
  return CoordinateSystem { origin, e1, e2 };
}

FrenetCart PathPlanner::createSplinePoint(double x, const tk::spline& spline) {
  return FrenetCart(Frenet { x, spline(x) });
}

#endif /* PATHPLANNER_H_ */
