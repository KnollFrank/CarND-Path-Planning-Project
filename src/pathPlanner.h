#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <math.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <tuple>

#include "coords/coordinateSystem.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "egoCar.h"
#include "funs.h"
#include "lane.h"
#include "path.h"
#include "pathCreator.h"
#include "previousData.h"
#include "spline.h"
#include "vehicle.h"

using namespace std;
using namespace std::experimental;

class PathPlanner {

 public:
  PathPlanner(const CoordsConverter& coordsConverter,
              const ReferencePoint& refPoint, const Lane& lane, double dt,
              double speed_limit_mph, const vector<Vehicle>& vehicles,
              const EgoCar& egoCar, const PreviousData& previousData);

  tuple<Path, Lane, ReferencePoint> createPath();

 private:
  bool isAnyVehicleWithin30MetersAheadOfEgoCarInLane(const Lane& lane);
  bool isAnyVehicleInLaneBehindOfEgoCarInTheWay(const Lane& lane);
  bool isVehicleWithin30MetersAheadOfEgoCarAtEndOfPath(const Vehicle& vehicle);
  bool isVehicleWithin30MetersAheadOfEgoCar(const double vehicle_s,
                                            const double egoCar_s) const;
  bool isVehicleWithin30MetersAheadOfEgoCarAtStartOfPath(
      const Vehicle& vehicle);
  double getNewVelocityMph(const bool tooClose, const double actualVelMph);
  Lane getNewLane(bool tooClose, const Lane& actualLane);
  Lane getMoreFreeLeftOrRightLane();
  std::experimental::optional<Vehicle> getNearestVehicleInLaneInFrontOfEgoCar(
      const Lane& lane);
  vector<Vehicle> getVehiclesInLaneInFrontOfEgoCar(const Lane& lane);
  bool canSwitch2Lane(const Lane& lane);
  std::vector<FrenetCart> createNewPoints();
  double getVehiclesSPositionAtEndOfPath(const Vehicle& vehicle);
  FrenetCart createFrenetCart(Frenet frenet) const;
  tuple<Lane, ReferencePoint> planBehaviour();
  tuple<Path, ReferencePoint> generatePath(const ReferencePoint& refPoint);

  const CoordsConverter& coordsConverter;
  const ReferencePoint& refPoint;
  const Lane& lane;
  const double dt;
  const double speed_limit_mph;
  const vector<Vehicle>& vehicles;
  const EgoCar& egoCarAtStartOfPath;
  EgoCar egoCarAtEndOfPath;
  const PreviousData& previousData;
};

PathPlanner::PathPlanner(const CoordsConverter& _coordsConverter,
                         const ReferencePoint& _refPoint, const Lane& _lane,
                         double _dt, double _speed_limit_mph,
                         const vector<Vehicle>& _vehicles,
                         const EgoCar& _egoCar,
                         const PreviousData& _previousData)
    : coordsConverter(_coordsConverter),
      refPoint(_refPoint),
      lane(_lane),
      dt(_dt),
      speed_limit_mph(_speed_limit_mph),
      vehicles(_vehicles),
      egoCarAtStartOfPath(_egoCar),
      egoCarAtEndOfPath(_egoCar),
      previousData(_previousData) {

  if (previousData.sizeOfPreviousPath() > 0) {
    egoCarAtEndOfPath.setPos(previousData.end_path);
  }
}

tuple<Path, Lane, ReferencePoint> PathPlanner::createPath() {
  Lane newLane;
  ReferencePoint refPointNew;
  tie(newLane, refPointNew) = planBehaviour();
  Path path;
  tie(path, refPointNew) = generatePath(refPointNew);
  return make_tuple(path, newLane, refPointNew);
}

tuple<Lane, ReferencePoint> PathPlanner::planBehaviour() {
  bool tooClose = isAnyVehicleWithin30MetersAheadOfEgoCarInLane(lane);
  Lane newLane = getNewLane(tooClose, lane);
  ReferencePoint refPointNew;
  refPointNew.vel_mph = getNewVelocityMph(tooClose, refPoint.vel_mph);
  refPointNew.point = egoCarAtEndOfPath.getPos();
  refPointNew.yaw_rad = deg2rad(egoCarAtEndOfPath.yaw_deg);
  return make_tuple(newLane, refPointNew);
}

tuple<Path, ReferencePoint> PathPlanner::generatePath(
    const ReferencePoint& refPoint) {

  PathCreator pathCreator(coordsConverter, egoCarAtEndOfPath, dt, refPoint);
  return pathCreator.createPath(previousData.previous_path, lane);
}

FrenetCart PathPlanner::createFrenetCart(Frenet frenet) const {
  return FrenetCart(frenet, coordsConverter);
}

bool PathPlanner::isAnyVehicleWithin30MetersAheadOfEgoCarInLane(
    const Lane& lane) {

  auto isVehicleWithin30MetersAheadOfEgoCarInLane =
      [&]
      (const Vehicle& vehicle) {
        return isVehicleInLane(vehicle, lane) && (isVehicleWithin30MetersAheadOfEgoCarAtStartOfPath(vehicle) || isVehicleWithin30MetersAheadOfEgoCarAtEndOfPath(vehicle));};

  return std::any_of(vehicles.cbegin(), vehicles.cend(),
                     isVehicleWithin30MetersAheadOfEgoCarInLane);
}

bool PathPlanner::isAnyVehicleInLaneBehindOfEgoCarInTheWay(const Lane& lane) {
  auto isVehicleBehindOfEgoCar =
      [&](const Vehicle& vehicle) {
        return vehicle.getPos().getFrenet().s < egoCarAtEndOfPath.getPos().getFrenet().s;
      };

  auto isVehicleInTheWay =
      [&](const Vehicle& vehicle) {
        return egoCarAtEndOfPath.getPos().getFrenet().s - vehicle.getPos().getFrenet().s < 30;
      };

  auto isVehicleInLaneBehindOfEgoCarInTheWay =
      [&](const Vehicle& vehicle) {
        return isVehicleInLane(vehicle, lane) && isVehicleBehindOfEgoCar(vehicle) && isVehicleInTheWay(vehicle);
      };

  return std::any_of(vehicles.cbegin(), vehicles.cend(),
                     isVehicleInLaneBehindOfEgoCarInTheWay);
}

bool PathPlanner::canSwitch2Lane(const Lane& lane) {
  return !isAnyVehicleWithin30MetersAheadOfEgoCarInLane(lane)
      && !isAnyVehicleInLaneBehindOfEgoCarInTheWay(lane);
}

double PathPlanner::getVehiclesSPositionAtEndOfPath(const Vehicle& vehicle) {
  const int numTimeSteps = previousData.sizeOfPreviousPath();
  const double time = numTimeSteps * dt;
  const double speed = vehicle.getVel_frenet_m_per_s().len();
  const double lenOfPath = time * speed;
  const double endOfPath = vehicle.getPos().getFrenet().s + lenOfPath;
  return endOfPath;
}

bool PathPlanner::isVehicleWithin30MetersAheadOfEgoCar(
    const double vehicle_s, const double egoCar_s) const {
  // TODO: replace magic number 30 with constant
  return vehicle_s > egoCar_s && vehicle_s - egoCar_s < 30;
}

bool PathPlanner::isVehicleWithin30MetersAheadOfEgoCarAtEndOfPath(
    const Vehicle& vehicle) {

  return isVehicleWithin30MetersAheadOfEgoCar(
      getVehiclesSPositionAtEndOfPath(vehicle),
      egoCarAtEndOfPath.getPos().getFrenet().s);
}

bool PathPlanner::isVehicleWithin30MetersAheadOfEgoCarAtStartOfPath(
    const Vehicle& vehicle) {

  return isVehicleWithin30MetersAheadOfEgoCar(
      vehicle.getPos().getFrenet().s,
      egoCarAtStartOfPath.getPos().getFrenet().s);
}

double PathPlanner::getNewVelocityMph(const bool tooClose,
                                      const double actualVelMph) {
  auto slowDown = [&]() {
    // TODO: nicht irgendwie bremsen mit 0.75, sondern genau so, dass ein Unfall mit dem vorderen Fahrzeug verhindert wird.
      const double newVelMph = actualVelMph - 0.75;
      return newVelMph > 0 ? newVelMph : actualVelMph;
    };

  auto speedUp = [&]() {
    const double newVelMph = actualVelMph + 0.4;
    return newVelMph < speed_limit_mph ? newVelMph : speed_limit_mph;
  };

  return tooClose ? slowDown() : speedUp();
}

vector<Vehicle> PathPlanner::getVehiclesInLaneInFrontOfEgoCar(
    const Lane& lane) {

  auto isInLane = [&](const Vehicle& vehicle) {
    return ::isInLane(vehicle.getPos().getFrenet().d, lane);
  };

  auto isInFrontOfEgoCar =
      [&](const Vehicle& vehicle) {
        return vehicle.getPos().getFrenet().s > egoCarAtEndOfPath.getPos().getFrenet().s;
      };

  return filter<Vehicle>(vehicles, [&](const Vehicle& vehicle) {
    return isInLane(vehicle) && isInFrontOfEgoCar(vehicle);
  });
}

std::experimental::optional<Vehicle> PathPlanner::getNearestVehicleInLaneInFrontOfEgoCar(
    const Lane& lane) {

  vector<Vehicle> vehiclesInLaneInFrontOfEgoCar =
      getVehiclesInLaneInFrontOfEgoCar(lane);

  auto isNearer = [](const Vehicle& vehicle1, const Vehicle& vehicle2) {
    return vehicle1.getPos().getFrenet().s < vehicle2.getPos().getFrenet().s;};

  return getOptionalMinimum<Vehicle>(vehiclesInLaneInFrontOfEgoCar, isNearer);
}

Lane PathPlanner::getMoreFreeLeftOrRightLane() {
  std::experimental::optional<Vehicle> leftVehicleInEgoCarsWay =
      getNearestVehicleInLaneInFrontOfEgoCar(Lane::LEFT);

  std::experimental::optional<Vehicle> rightVehicleInEgoCarsWay =
      getNearestVehicleInLaneInFrontOfEgoCar(Lane::RIGHT);

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

Lane PathPlanner::getNewLane(bool tooClose, const Lane& actualLane) {
  if (!tooClose) {
    return actualLane;
  }

  auto canSwitchFromLaneToLane = [&](const Lane& from, const Lane& to) {
    return actualLane == from && canSwitch2Lane(to);
  };

  if (canSwitchFromLaneToLane(Lane::LEFT, Lane::MIDDLE)) {
    return Lane::MIDDLE;
  }

  if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::LEFT)
      && canSwitchFromLaneToLane(Lane::MIDDLE, Lane::RIGHT)) {

    return getMoreFreeLeftOrRightLane();
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

  return actualLane;
}

#endif /* PATHPLANNER_H_ */
