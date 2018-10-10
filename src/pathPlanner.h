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
  PathPlanner(const CoordsConverter& coordsConverter, const ReferencePoint& refPoint,
              const Lane& lane, double dt, double speed_limit_mph,
              const vector<Vehicle>& vehicles, const EgoCar& egoCar,
              const PreviousData& previousData);

  tuple<Path, Lane, ReferencePoint> createPath();

 private:
  bool isEgoCarTooCloseToAnyVehicleInLane(const Lane& lane);
  bool willVehicleBeWithin30MetersAheadOfEgoCar(const Vehicle& vehicle);
  double getNewVelocity(bool too_close, double vel_mph);
  Lane getNewLane(bool too_close, const Lane& lane);
  Lane getMoreFreeLeftOrRightLane();
  std::experimental::optional<Vehicle> getNearestVehicleInLaneInFrontOfEgoCar(
      const Lane& lane);
  vector<Vehicle> getVehiclesInLaneInFrontOfEgoCar(const Lane& lane);
  bool canSwitch2Lane(const Lane& lane);
  std::vector<FrenetCart> createNewPoints();
  double getVehiclesSPositionAfterNumTimeSteps(const Vehicle& vehicle);
  FrenetCart createFrenetCart(Frenet frenet) const;
  tuple<Lane, ReferencePoint> planPath();
  tuple<Path, ReferencePoint> computePath(const ReferencePoint& refPoint);

  const CoordsConverter& coordsConverter;
  // TODO: refPoint und lane sollen unveränderbare Rückgabewerte von createPath sein.
  const ReferencePoint& refPoint;
  const Lane& lane;
  const double dt;
  const double speed_limit_mph;
  const vector<Vehicle>& vehicles;
  EgoCar egoCar;
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
      egoCar(_egoCar),
      previousData(_previousData) {
}

tuple<Path, Lane, ReferencePoint> PathPlanner::createPath() {
  Lane newLane;
  ReferencePoint refPointNew;
  tie(newLane, refPointNew) = planPath();
  Path path;
  tie(path, refPointNew) = computePath(refPointNew);
  return make_tuple(path, newLane, refPointNew);
}

tuple<Lane, ReferencePoint> PathPlanner::planPath() {
  if (previousData.sizeOfPreviousPath() > 0) {
    egoCar.setPos(createFrenetCart(previousData.end_path));
  }

  bool too_close = isEgoCarTooCloseToAnyVehicleInLane(lane);
  Lane newLane = getNewLane(too_close, lane);
  ReferencePoint refPointNew;
  refPointNew.vel_mph = getNewVelocity(too_close, refPoint.vel_mph);
  refPointNew.point = egoCar.getPos().getFrenet();
  refPointNew.yaw_rad = deg2rad(egoCar.yaw_deg);
  return make_tuple(newLane, refPointNew);
}

tuple<Path, ReferencePoint> PathPlanner::computePath(const ReferencePoint& refPoint) {
  PathCreator pathCreator(coordsConverter, previousData, egoCar, dt, lane);
  return pathCreator.createPath(refPoint);
}

FrenetCart PathPlanner::createFrenetCart(Frenet frenet) const {
  return FrenetCart(frenet, coordsConverter);
}

bool PathPlanner::isEgoCarTooCloseToAnyVehicleInLane(const Lane& lane) {
  auto isEgoCarTooCloseToVehicleInLane =
      [&]
      (const Vehicle& vehicle) {
        return isVehicleInLane(vehicle, lane) && willVehicleBeWithin30MetersAheadOfEgoCar(vehicle);};

  return std::any_of(vehicles.cbegin(), vehicles.cend(),
                     isEgoCarTooCloseToVehicleInLane);
}

bool PathPlanner::canSwitch2Lane(const Lane& lane) {
// TODO: hier den Pfad zur lane berechnen und simulieren, ob es mit irgendeinem Vehicle kracht.
  return !isEgoCarTooCloseToAnyVehicleInLane(lane);
}

double PathPlanner::getVehiclesSPositionAfterNumTimeSteps(
    const Vehicle& vehicle) {
// TODO: hier sollte man jeden einzelnen der numTimeSteps Schritte simulieren und bei jedem Schritt schauen, ob es kracht.
  const int numTimeSteps = previousData.sizeOfPreviousPath();
  const double speed = vehicle.getVel_frenet_m_per_s().len();
  return vehicle.getPos().getFrenet().s + numTimeSteps * dt * speed;
}

bool PathPlanner::willVehicleBeWithin30MetersAheadOfEgoCar(
    const Vehicle& vehicle) {
  double check_vehicle_s = getVehiclesSPositionAfterNumTimeSteps(vehicle);
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
    const Lane& lane) {

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

std::experimental::optional<Vehicle> PathPlanner::getNearestVehicleInLaneInFrontOfEgoCar(
    const Lane& lane) {

  vector<Vehicle> vehiclesInLaneInFrontOfEgoCar =
      getVehiclesInLaneInFrontOfEgoCar(lane);

  auto isNearer = [](const Vehicle& vehicle1, const Vehicle& vehicle2) {
    return vehicle1.getPos().getFrenet().s < vehicle2.getPos().getFrenet().s;};

  return getMinimum<Vehicle>(vehiclesInLaneInFrontOfEgoCar, isNearer);
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

Lane PathPlanner::getNewLane(bool too_close, const Lane& lane) {
  if (!too_close) {
    return lane;
  }

  auto canSwitchFromLaneToLane = [&](const Lane& from, const Lane& to) {
    return lane == from && canSwitch2Lane(to);
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

  return lane;
}

#endif /* PATHPLANNER_H_ */
