#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <math.h>
#include <algorithm>
#include <functional>
#include <iostream>
#include <iterator>
#include <vector>
#include <experimental/optional>

#include "alglib/interpolation.h"
#include "car.h"
#include "coords/cart.h"
#include "coords/coordinateSystem.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "funs.h"
#include "json.hpp"
#include "lane.h"
#include "path.h"
#include "previousData.h"

using namespace std;
using namespace std::experimental;

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
              Lane& lane, double dt, double speed_limit_mph);

  Path createPath(EgoCar egoCar, const PreviousData& previousData,
                  const vector<Vehicle>& vehicles);

 private:
  bool isEgoCarTooCloseToAnyVehicleInLane(const EgoCar& egoCar,
                                          const vector<Vehicle>& vehicles,
                                          const int numTimeSteps,
                                          const Lane& lane);
  bool willVehicleBeWithin30MetersAheadOfEgoCar(const EgoCar& egoCar,
                                                const Vehicle& vehicle,
                                                const int numTimeSteps);
  double getNewVelocity(bool too_close, double vel_mph);
  Lane getNewLane(bool too_close, const Lane& lane, const EgoCar& egoCar,
                  const vector<Vehicle>& vehicles, const int numTimeSteps);
  Lane getMoreFreeLeftOrRightLane(const EgoCar& egoCar,
                                  const vector<Vehicle>& vehicles);
  optional<Vehicle> getNearestVehicleInLaneInFrontOfEgoCar(
      const Lane& lane, const EgoCar& egoCar, const vector<Vehicle>& vehicles);
  vector<Vehicle> getVehiclesInLaneInFrontOfEgoCar(
      const Lane& lane, const EgoCar& egoCar, const vector<Vehicle>& vehicles);
  bool canSwitch2Lane(const EgoCar& egoCar, const Lane& lane,
                      const vector<Vehicle>& vehicles, const int numTimeSteps);
  CoordinateSystem createRotatedCoordinateSystem(const Frenet& origin,
                                                 double angle_rad);
  FrenetCart createSplinePoint(double x, const Spline& spline);
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
  std::vector<FrenetCart> createSplinePoints(const Spline& spline,
                                             const int num);
  vector<FrenetCart> transform(const CoordinateSystem& coordinateSystem,
                               const vector<FrenetCart>& points) const;
  std::vector<double> createSVals(const Spline& spline, const int num);
  void addPointsFromPreviousData(Path& path, const EgoCar& egoCar,
                                 const PreviousData& previousData);
  void addNewPoints(Path& path, const EgoCar& egoCar);
  double getVehiclesSPositionAfterNumTimeSteps(const Vehicle& vehicle,
                                               const int numTimeSteps);

  const CoordsConverter& coordsConverter;
  // TODO: refPoint und lane sollen unveränderbare Rückgabewerte von createPath sein.
  ReferencePoint& refPoint;
  Lane& lane;
  const double dt;
  const int path_size = 50;
  const double speed_limit_mph;
};

PathPlanner::PathPlanner(const CoordsConverter& _coordsConverter,
                         ReferencePoint& _refPoint, Lane& _lane, double _dt,
                         double _speed_limit_mph)
    : coordsConverter(_coordsConverter),
      refPoint(_refPoint),
      lane(_lane),
      dt(_dt),
      speed_limit_mph(_speed_limit_mph) {
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
            return createSplinePoints(carsPath.asSpline(), path_size - previousData.sizeOfPreviousPath());
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
    const EgoCar& egoCar, const vector<Vehicle>& vehicles,
    const int numTimeSteps, const Lane& lane) {

  auto isEgoCarTooCloseToVehicleInLane =
      [&]
      (const Vehicle& vehicle) {
        return isVehicleInLane(vehicle, lane) && willVehicleBeWithin30MetersAheadOfEgoCar(egoCar, vehicle, numTimeSteps);};

  return std::any_of(vehicles.cbegin(), vehicles.cend(),
                     isEgoCarTooCloseToVehicleInLane);
}

bool PathPlanner::canSwitch2Lane(const EgoCar& egoCar, const Lane& lane,
                                 const vector<Vehicle>& vehicles,
                                 const int numTimeSteps) {
  return !isEgoCarTooCloseToAnyVehicleInLane(egoCar, vehicles, numTimeSteps,
                                             lane);
}

double PathPlanner::getVehiclesSPositionAfterNumTimeSteps(
    const Vehicle& vehicle, const int numTimeSteps) {

  double speed = vehicle.getVel_frenet_m_per_s().len();
  return vehicle.getPos().getFrenet().s + numTimeSteps * dt * speed;
}

bool PathPlanner::willVehicleBeWithin30MetersAheadOfEgoCar(
    const EgoCar& egoCar, const Vehicle& vehicle, const int numTimeSteps) {
  double check_vehicle_s = getVehiclesSPositionAfterNumTimeSteps(vehicle,
                                                                 numTimeSteps);
  // TODO: replace magic number 30 with constant
  return check_vehicle_s > egoCar.getPos().getFrenet().s
      && check_vehicle_s - egoCar.getPos().getFrenet().s < 30;
}

double PathPlanner::getNewVelocity(bool too_close, double vel_mph) {
  double speed_delta_mph = 2;

  if (too_close) {
    vel_mph -= speed_delta_mph;
  } else if (vel_mph + speed_delta_mph < speed_limit_mph) {
    vel_mph += speed_delta_mph;
  } else {
    vel_mph = speed_limit_mph;
  }

  return vel_mph;
}

vector<Vehicle> PathPlanner::getVehiclesInLaneInFrontOfEgoCar(
    const Lane& lane, const EgoCar& egoCar, const vector<Vehicle>& vehicles) {

  auto isInLane = [&](const Vehicle& vehicle) {
    return ::isInLane(vehicle.getPos().getFrenet().d, lane);
  };

  auto isInFrontOfEgoCar = [&](const Vehicle& vehicle) {
    return vehicle.getPos().getFrenet().s > egoCar.getPos().getFrenet().s;
  };

  return filter<Vehicle>(vehicles, [&](const Vehicle& vehicle) {
    return isInLane(vehicle) && isInFrontOfEgoCar(vehicle);
  });
}

optional<Vehicle> PathPlanner::getNearestVehicleInLaneInFrontOfEgoCar(
    const Lane& lane, const EgoCar& egoCar, const vector<Vehicle>& vehicles) {

  vector<Vehicle> vehiclesInLaneInFrontOfEgoCar =
      getVehiclesInLaneInFrontOfEgoCar(lane, egoCar, vehicles);

  auto isNearer = [](const Vehicle& vehicle1, const Vehicle& vehicle2) {
    return vehicle1.getPos().getFrenet().s < vehicle2.getPos().getFrenet().s;};

  return getMinimum<Vehicle>(vehiclesInLaneInFrontOfEgoCar, isNearer);
}

Lane PathPlanner::getMoreFreeLeftOrRightLane(const EgoCar& egoCar,
                                             const vector<Vehicle>& vehicles) {

  optional<Vehicle> leftVehicleInEgoCarsWay =
      getNearestVehicleInLaneInFrontOfEgoCar(Lane::LEFT, egoCar, vehicles);

  optional<Vehicle> rightVehicleInEgoCarsWay =
      getNearestVehicleInLaneInFrontOfEgoCar(Lane::RIGHT, egoCar, vehicles);

  if (leftVehicleInEgoCarsWay && rightVehicleInEgoCarsWay) {
    bool isLeftLaneMoreFree = (*leftVehicleInEgoCarsWay).getPos().getFrenet().s
        > (*rightVehicleInEgoCarsWay).getPos().getFrenet().s;
    return isLeftLaneMoreFree ? Lane::LEFT : Lane::RIGHT;
  }

  if (leftVehicleInEgoCarsWay && !rightVehicleInEgoCarsWay) {
    return Lane::RIGHT;
  }

  if (!leftVehicleInEgoCarsWay && rightVehicleInEgoCarsWay) {
    return Lane::LEFT;
  }

  if (!leftVehicleInEgoCarsWay && !rightVehicleInEgoCarsWay) {
    return Lane::LEFT;
  }
}

Lane PathPlanner::getNewLane(bool too_close, const Lane& lane,
                             const EgoCar& egoCar,
                             const vector<Vehicle>& vehicles,
                             const int numTimeSteps) {
  if (!too_close) {
    return lane;
  }

  auto canSwitchFromLaneToLane = [&](const Lane& from, const Lane& to) {
    return lane == from && canSwitch2Lane(egoCar, to, vehicles, numTimeSteps);
  };

  if (canSwitchFromLaneToLane(Lane::LEFT, Lane::MIDDLE)) {
    return Lane::MIDDLE;
  }

  if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::LEFT)
      && canSwitchFromLaneToLane(Lane::MIDDLE, Lane::RIGHT)) {

    return getMoreFreeLeftOrRightLane(egoCar, vehicles);
  }

  if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::LEFT)) {
    return Lane::LEFT;
  }

  if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::RIGHT)) {
    return Lane::RIGHT;
  }

  if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::LEFT)) {
    return Lane::LEFT;
  }

  if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::RIGHT)) {
    return Lane::RIGHT;
  }

  if (canSwitchFromLaneToLane(Lane::RIGHT, Lane::MIDDLE)) {
    return Lane::MIDDLE;
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
  auto egoCarPlus =
      [&](int s_offset) {
        return FrenetCart(
            Frenet {egoCar.getPos().getFrenet().s + s_offset, getMiddleOfLane(lane)},
            coordsConverter);
      };

  return {egoCarPlus(30), egoCarPlus(60), egoCarPlus(90)};
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

// TODO: hier sollen s_vals erzeugt werden, die einen Abstand nach der Bogenlänge s_delta der Splinekurve spline haben.
vector<double> PathPlanner::createSVals(const Spline& spline, const int num) {
  vector<double> s_vals;
  const double s_delta = dt * mph2meter_per_sec(0.89 * refPoint.vel_mph);
  for (int i = 0; i < num; i++) {
    s_vals.push_back((i + 1) * s_delta);
  }
  return s_vals;
}

vector<FrenetCart> PathPlanner::createSplinePoints(const Spline& spline,
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

FrenetCart PathPlanner::createSplinePoint(double x, const Spline& spline) {
  return FrenetCart(Frenet { x, spline(x) }, coordsConverter);
}

#endif /* PATHPLANNER_H_ */
