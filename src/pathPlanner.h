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
#include "coords/frenet.h"

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
  Frenet createSplinePoint(double x, const tk::spline& spline);
  void sort_and_remove_duplicates(vector<Frenet>& points);
  std::vector<Frenet> createPointsFromPreviousData(
      const EgoCar& egoCar, const PreviousData& previousData);
  void appendSnd2Fst(vector<Frenet>& fst, const vector<Frenet>& snd);
  std::vector<Frenet> createNewPoints(const EgoCar& egoCar);
  vector<Frenet> workWithPathInCarsCoordinateSystem(
      const Path& path,
      const function<vector<Frenet>(const Path& carsPath)>& transformCarsPath2Points);
  vector<Frenet> enterCarsCoordinateSystem(const Frenet& origin,
                                           const double angle_rad,
                                           const vector<Frenet>& points);
  vector<Frenet> leaveCarsCoordinateSystem(const Frenet& origin,
                                           double angle_rad,
                                           const vector<Frenet>& points);
  std::vector<Frenet> createSplinePoints(const tk::spline& spline,
                                         const int num);
  vector<Frenet> transform(const CoordinateSystem& coordinateSystem,
                           const vector<Frenet>& points) const;
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

vector<Frenet> PathPlanner::enterCarsCoordinateSystem(
    const Frenet& origin, const double angle_rad,
    const vector<Frenet>& points) {
  CoordinateSystem coordinateSystem = createRotatedCoordinateSystem(Frenet { 0,
                                                                        0 },
                                                                    angle_rad);
  vector<Frenet> origin2points = map2<Frenet, Frenet>(points,
                                                      [&](const Frenet& point) {
                                                        return point - origin;
                                                      });
  return transform(coordinateSystem, origin2points);
}

vector<Frenet> PathPlanner::leaveCarsCoordinateSystem(
    const Frenet& origin, double angle_rad, const vector<Frenet>& points) {
  return transform(createRotatedCoordinateSystem(origin, angle_rad), points);
}

vector<Frenet> PathPlanner::workWithPathInCarsCoordinateSystem(
    const Path& path,
    const function<vector<Frenet>(const Path& carsPath)>& transformCarsPath2Points) {

  Path carsPath;
  carsPath.points = enterCarsCoordinateSystem(refPoint.point, -refPoint.yaw_rad,
                                              path.points);
  sort_and_remove_duplicates(carsPath.points);
  vector<Frenet> points = transformCarsPath2Points(carsPath);
  return leaveCarsCoordinateSystem(refPoint.point, refPoint.yaw_rad, points);
}

Path PathPlanner::createPath(EgoCar egoCar, const PreviousData& previousData,
                             const vector<Vehicle>& vehicles) {

  // printInfo(egoCar, vehicles);

  if (previousData.sizeOfPreviousPath() > 0) {
    egoCar.setPos_frenet(Frenet { previousData.end_path.s,
        egoCar.getPos_frenet().d });
  }

  bool too_close = isEgoCarTooCloseToAnyVehicleInLane(
      egoCar, vehicles, previousData.sizeOfPreviousPath());
  lane = getNewLane(too_close, lane);
  refPoint.vel_mph = getNewVelocity(too_close, refPoint.vel_mph);
  refPoint.point = egoCar.getPos_frenet();
  refPoint.yaw_rad = deg2rad(egoCar.yaw_deg);

  Path path;
  addPointsFromPreviousData(path, egoCar, previousData);
  addNewPoints(path, egoCar);

  vector<Frenet> points = workWithPathInCarsCoordinateSystem(
      path, [&](const Path& carsPath) {
        return createSplinePoints(
            // TODO: diesen Spline später in XY-Koordinaten erzeugen, damit es nicht ruckelt im Simulator.
            // Man muß einen Path sowohl in Frenet als auch in kartesischen Koordinaten erhalten können, und zwar
            // nicht über den ungenauen CoordsConverter, sondern über ein Runnable.
            carsPath.asSpline(), path_size - previousData.sizeOfPreviousPath());
      });

  Path next_vals;
  appendSnd2Fst(next_vals.points, previousData.previous_path.points);
  appendSnd2Fst(next_vals.points, points);

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

vector<Frenet> PathPlanner::createPointsFromPreviousData(
    const EgoCar& egoCar, const PreviousData& previousData) {

  vector<Frenet> points;
  if (previousData.sizeOfPreviousPath() < 2) {
    Frenet prev = egoCar.getPos_frenet()
        - Frenet::fromAngle(deg2rad(egoCar.yaw_deg));
    points.push_back(prev);
    points.push_back(egoCar.getPos_frenet());
  } else {
    refPoint.point = previousData.previous_path.points[previousData
        .sizeOfPreviousPath() - 1];
    Frenet prev = previousData.previous_path.points[previousData
        .sizeOfPreviousPath() - 2];
    refPoint.yaw_rad = (refPoint.point - prev).getHeading();
    points.push_back(prev);
    points.push_back(refPoint.point);
  }
  return points;
}

void PathPlanner::appendSnd2Fst(vector<Frenet>& fst,
                                const vector<Frenet>& snd) {
  fst.insert(std::end(fst), std::begin(snd), std::end(snd));
}

vector<Frenet> PathPlanner::createNewPoints(const EgoCar& egoCar) {
  vector<Frenet> points;
  points.push_back(
      Frenet { egoCar.getPos_frenet().s + 30, getMiddleOfLane(lane) });
  points.push_back(
      Frenet { egoCar.getPos_frenet().s + 60, getMiddleOfLane(lane) });
  points.push_back(
      Frenet { egoCar.getPos_frenet().s + 90, getMiddleOfLane(lane) });
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

void PathPlanner::sort_and_remove_duplicates(vector<Frenet>& points) {
  std::sort(points.begin(), points.end(),
            [](const Frenet& p1, const Frenet& p2) {return p1.s < p2.s;});
  points.erase(
      unique(points.begin(), points.end(),
             [](const Frenet& p1, const Frenet& p2) {return p1.s == p2.s;}),
      points.end());
}

vector<Frenet> PathPlanner::transform(const CoordinateSystem& coordinateSystem,
                                      const vector<Frenet>& points) const {
  return map2<Frenet, Frenet>(points, [&](const Frenet& point) {
    return coordinateSystem.transform(point);
  });
}

vector<double> PathPlanner::createSVals(const tk::spline& spline, const int num) {
  vector<double> s_vals;
  Frenet target = createSplinePoint(30.0, spline);
  double s_add_on = 0;
  double N = target.len() / (dt * mph2meter_per_sec(refPoint.vel_mph));
  for (int i = 0; i < num; i++) {
    s_add_on += target.s / N;
    s_vals.push_back(s_add_on);
  }
  return s_vals;
}

vector<Frenet> PathPlanner::createSplinePoints(const tk::spline& spline,
                                               const int num) {

  vector<double> s_vals = createSVals(spline, num);
  vector<Frenet> points = map2<double, Frenet>(
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

Frenet PathPlanner::createSplinePoint(double x, const tk::spline& spline) {
  return Frenet { x, spline(x) };
}

#endif /* PATHPLANNER_H_ */
