#ifndef TESTS_SIMULATOR2_H_
#define TESTS_SIMULATOR2_H_

#include <gtest/gtest.h>
#include <gtest/gtest-message.h>
#include <algorithm>
#include <iostream>
#include <vector>

#include "../coords/cart.h"
#include "../coords/coordsConverter.h"
#include "../coords/frenet.h"
#include "../coords/frenetCart.h"
#include "../egoCar.h"
#include "../funs.h"
#include "../lane.h"
#include "../path.h"
#include "../pathPlanner.h"
#include "../rectangle.h"
#include "../referencePoint.h"
#include "../vehicle.h"
#include "vehicleDriver.h"

using namespace std;
using namespace std::experimental;

class Simulator2 {

 public:
  Simulator2(const CoordsConverter& _coordsConverter, EgoCar& _egoCar,
             vector<Vehicle>& _vehicles, double _dt,
             VehicleDriver* _vehicleDriver)
      : coordsConverter(_coordsConverter),
        egoCar(_egoCar),
        vehicles(_vehicles),
        dt(_dt),
        vehicleDriver(_vehicleDriver) {
  }

  ~Simulator2() {
    delete vehicleDriver;
  }

  bool driveEgoCarAndVehicles(const Path& path) {
    return drive2PointsOfEgoCarAndDriveVehicles(path.points);
  }

  bool drive2PointsOfEgoCarAndDriveVehicles(const vector<FrenetCart>& points) {

    for (int i = 0; i < points.size(); i++) {
      driveVehicles();
      bool incidentHappened = drive2PointOfEgoCar(points[i]);
      if (incidentHappened) {
        return true;
      }
    }

    return false;
  }

  void driveVehicles() {
    for (Vehicle& vehicle : vehicles) {
      driveVehicle(vehicle);
    }
  }

  void driveVehicle(Vehicle& vehicle) {
    vehicleDriver->driveVehicle(vehicle, egoCar, dt);
  }

  bool drive2PointOfEgoCar(const FrenetCart& dst) {
    const FrenetCart& src = egoCar.getPos();
    egoCar.speed_mph = meter_per_sec2mph(
        src.getXY().distanceTo(dst.getXY()) / dt);
    egoCar.setPos(dst);
    egoCar.yaw_deg = rad2deg((dst.getFrenet() - src.getFrenet()).getHeading());

    return isCollision(egoCar, vehicles);
  }

  bool isCollision(const EgoCar& egoCar, const Vehicle& vehicle) {
    return egoCar.getShape().overlaps(vehicle.getShape());
  }

  bool isCollision(const EgoCar& egoCar, const vector<Vehicle>& vehicles) {
    return bool(getCollidingVehicle(egoCar, vehicles));
  }

  std::experimental::optional<Vehicle> getCollidingVehicle(
      const EgoCar& egoCar, const vector<Vehicle>& vehicles) {

    auto collidingVehicleIterator = std::find_if(
        vehicles.cbegin(), vehicles.cend(),
        [&](const Vehicle& vehicle) {return isCollision(egoCar, vehicle);});
    return
        collidingVehicleIterator != vehicles.cend() ?
            std::experimental::make_optional(*collidingVehicleIterator) :
            std::experimental::nullopt;
  }

  const CoordsConverter& coordsConverter;
  EgoCar& egoCar;
  vector<Vehicle>& vehicles;
  double dt;
  VehicleDriver* vehicleDriver;
};

#endif
