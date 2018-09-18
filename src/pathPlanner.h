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
  Path createPoints(const EgoCar& egoCar, const PreviousData& previousData);
  Path createNextVals(const Path& path, const PreviousData& previousData);
  Lane getNewLane(bool too_close, Lane lane);
  CoordinateSystem createRotatedCoordinateSystem(const Point& origin,
                                                 double angle_rad);
  Point createSplinePoint(double x, const tk::spline& s);
  void sort_and_remove_duplicates(vector<Point>& points);
  std::vector<Point> createPointsFromPreviousData(
      const EgoCar& egoCar, const PreviousData& previousData);
  void appendSnd2Fst(vector<Point>& fst, const vector<Point>& snd);
  std::vector<Point> createNewPoints(const EgoCar& egoCar);
  void rotate(vector<Point>& points, const Point& center,
              const double angle_rad);
  std::vector<Point> createTransformedSplinePoints(const tk::spline& s,
                                                   const int num);
  vector<Point> transform(const CoordinateSystem& coordinateSystem,
                          const vector<Point>& points) const;
  std::vector<double> createXVals(const tk::spline& s, const int num);

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
  refPoint.point = egoCar.getPos_cart();
  refPoint.yaw_rad = deg2rad(egoCar.yaw_deg);

  Path path = createPoints(egoCar, previousData);
  return createNextVals(path, previousData);
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

vector<Point> PathPlanner::createPointsFromPreviousData(
    const EgoCar& egoCar, const PreviousData& previousData) {

  vector<Point> points;
  if (previousData.sizeOfPreviousPath() < 2) {
    Point prev = egoCar.getPos_cart()
        - Point::fromAngle(deg2rad(egoCar.yaw_deg));
    points.push_back(prev);
    points.push_back(egoCar.getPos_cart());
  } else {
    refPoint.point = previousData.previous_path.points[previousData
        .sizeOfPreviousPath() - 1];
    Point prev = previousData.previous_path.points[previousData
        .sizeOfPreviousPath() - 2];
    refPoint.yaw_rad = (refPoint.point - prev).getHeading();
    points.push_back(prev);
    points.push_back(refPoint.point);
  }
  return points;
}

void PathPlanner::appendSnd2Fst(vector<Point>& fst, const vector<Point>& snd) {
  fst.insert(std::end(fst), std::begin(snd), std::end(snd));
}

vector<Point> PathPlanner::createNewPoints(const EgoCar& egoCar) {
  vector<Point> points;
  points.push_back(coordsConverter.getXY(Frenet { egoCar.getPos_frenet().s + 30,
      getMiddleOfLane(lane) }));
  points.push_back(coordsConverter.getXY(Frenet { egoCar.getPos_frenet().s + 60,
      getMiddleOfLane(lane) }));
  points.push_back(coordsConverter.getXY(Frenet { egoCar.getPos_frenet().s + 90,
      getMiddleOfLane(lane) }));
  return points;
}

void PathPlanner::rotate(vector<Point>& points, const Point& center,
                         const double angle_rad) {
  CoordinateSystem coordinateSystem = createRotatedCoordinateSystem(
      Point { 0, 0 }, angle_rad);
  mapInPlace(points, [&](const Point& point) {
    return coordinateSystem.transform(point - center);
  });
}

Path PathPlanner::createPoints(const EgoCar& egoCar,
                               const PreviousData& previousData) {
  Path path;
  appendSnd2Fst(path.points,
                createPointsFromPreviousData(egoCar, previousData));
  appendSnd2Fst(path.points, createNewPoints(egoCar));
  rotate(path.points, refPoint.point, -refPoint.yaw_rad);
  sort_and_remove_duplicates(path.points);
  return path;
}

void PathPlanner::sort_and_remove_duplicates(vector<Point>& points) {
  std::sort(points.begin(), points.end(),
            [](const Point& p1, const Point& p2) {return p1.x < p2.x;});
  points.erase(
      unique(points.begin(), points.end(),
             [](const Point& p1, const Point& p2) {return p1.x == p2.x;}),
      points.end());
}

vector<Point> PathPlanner::transform(const CoordinateSystem& coordinateSystem,
                                     const vector<Point>& points) const {
  return map2<Point, Point>(points, [&](const Point& point) {
    return coordinateSystem.transform(point);
  });
}

// TODO: refactor
vector<double> PathPlanner::createXVals(const tk::spline& s, const int num) {
  vector<double> x_vals;
  Point target = createSplinePoint(30.0, s);
  double x_add_on = 0;
  double N = target.len() / (dt * mph2meter_per_sec(refPoint.vel_mph));
  for (int i = 0; i < num; i++) {
    x_add_on += target.x / N;
    x_vals.push_back(x_add_on);
  }
  return x_vals;
}

// TODO: refactor. erst Werte x_add_on + target.x / N erzeugen, dann mit map createSplinePoint(), dann transform()
vector<Point> PathPlanner::createTransformedSplinePoints(const tk::spline& s,
                                                         const int num) {

  vector<double> x_vals = createXVals(s, num);
  vector<Point> points = map2<double, Point>(
      x_vals, [&](const double x_val) {return createSplinePoint(x_val, s);});

  CoordinateSystem coordinateSystem = createRotatedCoordinateSystem(
      refPoint.point, refPoint.yaw_rad);
  return transform(coordinateSystem, points);
}

// TODO: refactor
Path PathPlanner::createNextVals(const Path& path,
                                 const PreviousData& previousData) {
  Path next_vals;
  tk::spline s = path.asSpline();
  appendSnd2Fst(next_vals.points, previousData.previous_path.points);
  appendSnd2Fst(
      next_vals.points,
      createTransformedSplinePoints(
          s, path_size - previousData.sizeOfPreviousPath()));

  return next_vals;
}

Lane PathPlanner::getNewLane(bool too_close, Lane lane) {
  if (too_close && lane > Lane::LEFT) {
    lane = Lane::LEFT;
  }

  return lane;
}

CoordinateSystem PathPlanner::createRotatedCoordinateSystem(const Point& origin,
                                                            double angle_rad) {
  Point e1 = Point { cos(angle_rad), sin(angle_rad) };
  Point e2 = Point { -sin(angle_rad), cos(angle_rad) };
  return CoordinateSystem { origin, e1, e2 };
}

Point PathPlanner::createSplinePoint(double x, const tk::spline& s) {
  return Point { x, s(x) };
}

#endif /* PATHPLANNER_H_ */
