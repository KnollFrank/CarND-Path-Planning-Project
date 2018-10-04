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

void printInfo(const EgoCar& egoCar, const vector<Vehicle>& vehicles,
               const CoordsConverter& coordsConverter) {
  auto isCloserToEgoCar =
      [&](const Vehicle& vehicle1, const Vehicle& vehicle2) {
        double distance1 = egoCar.getPos().getXY().distanceTo(vehicle1.getPos().getXY());
        double distance2 = egoCar.getPos().getXY().distanceTo(vehicle2.getPos().getXY());
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
                                          const int prev_size,
                                          const Lane& lane);
  bool willVehicleBeWithin30MetersAheadOfEgoCar(const EgoCar& egoCar,
                                                const Vehicle& vehicle,
                                                const int prev_size);
  double getNewVelocity(bool too_close, double vel_mph);
  Lane getNewLane(bool too_close, Lane lane, const EgoCar& egoCar,
                  const vector<Vehicle>& vehicles, const int prev_size);
  bool canSwitch2Lane(const EgoCar& egoCar, const Lane& lane,
                      const vector<Vehicle>& vehicles, const int prev_size);
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
        return FrenetCart(point.getFrenet() - origin, coordsConverter);
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
    egoCar.setPos(FrenetCart(previousData.end_path, coordsConverter));
  }

  bool too_close = isEgoCarTooCloseToAnyVehicleInLane(
      egoCar, vehicles, previousData.sizeOfPreviousPath(), lane);
  lane = getNewLane(too_close, lane, egoCar, vehicles,
                    previousData.sizeOfPreviousPath());
  refPoint.vel_mph = getNewVelocity(too_close, refPoint.vel_mph);
  refPoint.point = egoCar.getPos().getFrenet();
  refPoint.yaw_rad = deg2rad(egoCar.yaw_deg);

  Path path;
  addPointsFromPreviousData(path, egoCar, previousData);
  addNewPoints(path, egoCar);

  vector<FrenetCart> points =
      workWithPathInCarsCoordinateSystem(
          path,
          [&](const Path& carsPath) {
            return createSplinePoints(carsPath.asSpline(coordsConverter), path_size - previousData.sizeOfPreviousPath());
          });

  Path next_vals;
  appendSnd2Fst(next_vals.points, previousData.previous_path.points);
  appendSnd2Fst(next_vals.points, points);

  return next_vals;

//  {
//    Path next_vals;
//    double dist_inc = 0.5;
//    for (int i = 0; i < 50; i++) {
//      double next_s = egoCar.getPos_frenet().s + dist_inc * (i + 1);
//      double next_d = 6;
//      next_vals.points.push_back(FrenetCart(Frenet { next_s, next_d }));
//    }
//    return next_vals;
//  }
}

bool PathPlanner::isEgoCarTooCloseToAnyVehicleInLane(
    const EgoCar& egoCar, const vector<Vehicle>& vehicles, const int prev_size,
    const Lane& lane) {
  auto isEgoCarTooCloseToVehicleInLane =
      [&]
      (const Vehicle& vehicle) {
        return isVehicleInLane(vehicle, lane) && willVehicleBeWithin30MetersAheadOfEgoCar(egoCar, vehicle, prev_size);};

  return std::any_of(vehicles.cbegin(), vehicles.cend(),
                     isEgoCarTooCloseToVehicleInLane);
}

bool PathPlanner::canSwitch2Lane(const EgoCar& egoCar, const Lane& lane,
                                 const vector<Vehicle>& vehicles,
                                 const int prev_size) {
  return !isEgoCarTooCloseToAnyVehicleInLane(egoCar, vehicles, prev_size, lane);
}

bool PathPlanner::willVehicleBeWithin30MetersAheadOfEgoCar(
    const EgoCar& egoCar, const Vehicle& vehicle, const int prev_size) {
  double check_speed = vehicle.getVel_cart_m_per_s().len();
  double check_vehicle_s = vehicle.getPos().getFrenet().s
      + prev_size * dt * check_speed;
  // TODO: replace magic number 30 with constant
  return check_vehicle_s > egoCar.getPos().getFrenet().s
      && check_vehicle_s - egoCar.getPos().getFrenet().s < 30;
}

double PathPlanner::getNewVelocity(bool too_close, double vel_mph) {
  if (too_close || vel_mph > 50) {
    vel_mph -= .224;
  } else if (vel_mph < 49.5) {
    vel_mph += .224;
  }

  return vel_mph;
}

// TODO: refactor
Lane PathPlanner::getNewLane(bool too_close, Lane lane, const EgoCar& egoCar,
                             const vector<Vehicle>& vehicles,
                             const int prev_size) {
  if (!too_close) {
    return lane;
  }

  if (lane == Lane::LEFT
      && canSwitch2Lane(egoCar, Lane::MIDDLE, vehicles, prev_size)) {
    lane = Lane::MIDDLE;
  }

  if (lane == Lane::MIDDLE
      && canSwitch2Lane(egoCar, Lane::LEFT, vehicles, prev_size)) {
    lane = Lane::LEFT;
  }

  if (lane == Lane::MIDDLE
      && canSwitch2Lane(egoCar, Lane::RIGHT, vehicles, prev_size)) {
    lane = Lane::RIGHT;
  }

  if (lane == Lane::RIGHT
      && canSwitch2Lane(egoCar, Lane::MIDDLE, vehicles, prev_size)) {
    lane = Lane::MIDDLE;
  }

  return lane;
}

vector<FrenetCart> PathPlanner::createPointsFromPreviousData(
    const EgoCar& egoCar, const PreviousData& previousData) {

  vector<FrenetCart> points;
  if (previousData.sizeOfPreviousPath() < 2) {
    Frenet prev = egoCar.getPos().getFrenet()
        - Frenet::fromAngle(deg2rad(egoCar.yaw_deg));
    points.push_back(FrenetCart(prev, coordsConverter));
    points.push_back(FrenetCart(egoCar.getPos().getFrenet(), coordsConverter));
  } else {
    refPoint.point = previousData.previous_path.points[previousData
        .sizeOfPreviousPath() - 1].getFrenet();
    Frenet prev = previousData.previous_path.points[previousData
        .sizeOfPreviousPath() - 2].getFrenet();
    refPoint.yaw_rad = (refPoint.point - prev).getHeading();
    points.push_back(FrenetCart(prev, coordsConverter));
    points.push_back(FrenetCart(refPoint.point, coordsConverter));
  }
  return points;
}

vector<FrenetCart> PathPlanner::createNewPoints(const EgoCar& egoCar) {
  auto createNewPoint = [&](int s_offset) {
    return FrenetCart(
        Frenet {egoCar.getPos().getFrenet().s + s_offset,
          getMiddleOfLane(lane)},
        coordsConverter);
  };

  return {createNewPoint(30), createNewPoint(60), createNewPoint( 90)};
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
      [&](const FrenetCart& p1, const FrenetCart& p2) {return p1.getFrenet().s < p2.getFrenet().s;});
  points.erase(
      unique(
          points.begin(),
          points.end(),
          [&](const FrenetCart& p1, const FrenetCart& p2) {return p1.getFrenet().s == p2.getFrenet().s;}),
      points.end());
}

vector<FrenetCart> PathPlanner::transform(
    const CoordinateSystem& coordinateSystem,
    const vector<FrenetCart>& points) const {

  return map2<FrenetCart, FrenetCart>(
      points,
      [&](const FrenetCart& point) {
        return FrenetCart(coordinateSystem.transform(point.getFrenet()), coordsConverter);});
}

vector<double> PathPlanner::createSVals(const tk::spline& spline,
                                        const int num) {
  vector<double> s_vals;
  Frenet target = createSplinePoint(30.0, spline).getFrenet();
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
  return FrenetCart(Frenet { x, spline(x) }, coordsConverter);
}

#endif /* PATHPLANNER_H_ */
