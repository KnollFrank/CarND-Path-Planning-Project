#ifndef TESTS_SIMULATOR_H_
#define TESTS_SIMULATOR_H_

#include <gtest/gtest.h>
#include <gtest/gtest-message.h>
#include <iostream>
#include <vector>

#include "../car.h"
#include "../coords/cart.h"
#include "../coords/coordsConverter.h"
#include "../coords/frenet.h"
#include "../coords/frenetCart.h"
#include "../funs.h"
#include "../lane.h"
#include "../path.h"
#include "../pathPlanner.h"
#include "../previousData.h"

using namespace std;
using namespace std::experimental;

#define GTEST_COUT std::cerr

class Simulator {
 public:
  Simulator(ReferencePoint& refPoint, Lane& lane,
            const CoordsConverter& coordsConverter, EgoCar& egoCar,
            PreviousData& previousData, vector<Vehicle>& vehicles, double dt,
            std::experimental::optional<int> minSecs2Drive);
  void drive(function<void(void)> afterEachMovementOfEgoCar);
  static bool isCollision(const EgoCar& egoCar, const Vehicle& vehicle);
  static bool isCollision(const EgoCar& egoCar,
                          const vector<Vehicle>& vehicles);
  static bool oneRoundDriven(const EgoCar& egoCar);

 private:
  double driveEgoCarAndVehicles(function<void(void)> afterEachMovementOfEgoCar);
  double drive2PointsOfEgoCarAndDriveVehicles(
      const vector<FrenetCart>& points, int numberOfUnprocessedPathElements,
      function<void(void)> afterEachMovementOfEgoCar);
  void driveVehicles();
  void driveVehicle(Vehicle& vehicle);
  void drive2PointOfEgoCar(const Frenet& dst,
                           function<void(void)> afterEachMovementOfEgoCar);
  void updatePreviousData(const vector<FrenetCart>& points,
                          int numberOfUnprocessedPathElements,
                          const Path& path);
  bool oneRoundDriven();
  void assertNoIncidentsHappened();

  ReferencePoint& refPoint;
  Lane& lane;
  const CoordsConverter& coordsConverter;
  EgoCar& egoCar;
  PreviousData& previousData;
  vector<Vehicle>& vehicles;
  double dt;
  std::experimental::optional<int> minSecs2Drive;
  function<void(void)> afterEachMovementOfEgoCar;
};

Simulator::Simulator(ReferencePoint& _refPoint, Lane& _lane,
                     const CoordsConverter& _coordsConverter, EgoCar& _egoCar,
                     PreviousData& _previousData, vector<Vehicle>& _vehicles,
                     double _dt,
                     std::experimental::optional<int> _minSecs2Drive)
    : refPoint(_refPoint),
      lane(_lane),
      coordsConverter(_coordsConverter),
      egoCar(_egoCar),
      previousData(_previousData),
      vehicles(_vehicles),
      dt(_dt),
      minSecs2Drive(_minSecs2Drive) {
}

void Simulator::drive(function<void(void)> afterEachMovementOfEgoCar) {
  double secsDriven = 0;
  while ((!minSecs2Drive || secsDriven <= minSecs2Drive.value())
      && !oneRoundDriven()) {
    secsDriven += driveEgoCarAndVehicles(afterEachMovementOfEgoCar);
  }
}

bool Simulator::oneRoundDriven(const EgoCar& egoCar) {
  return egoCar.getPos_frenet().s > 6900;
}

bool Simulator::oneRoundDriven() {
  return oneRoundDriven(egoCar);
}

double Simulator::driveEgoCarAndVehicles(
    function<void(void)> afterEachMovementOfEgoCar) {
  PathPlanner pathPlanner(coordsConverter, refPoint, lane, dt);
  Path path = pathPlanner.createPath(egoCar, previousData, vehicles);
  int numberOfUnprocessedPathElements = 10;
  double secsDriven = drive2PointsOfEgoCarAndDriveVehicles(
      path.points, numberOfUnprocessedPathElements, afterEachMovementOfEgoCar);
  updatePreviousData(path.points, numberOfUnprocessedPathElements, path);
  return secsDriven;
}

void Simulator::updatePreviousData(const vector<FrenetCart>& points,
                                   int numberOfUnprocessedPathElements,
                                   const Path& path) {
  previousData.previous_path.points.clear();
  for (int i = points.size() - numberOfUnprocessedPathElements;
      i < points.size(); i++) {
    previousData.previous_path.points.push_back(path.points[i]);
  }
  previousData.end_path = points[points.size() - numberOfUnprocessedPathElements
      - 1].getFrenet(coordsConverter);
}

double Simulator::drive2PointsOfEgoCarAndDriveVehicles(
    const vector<FrenetCart>& points, int numberOfUnprocessedPathElements,
    function<void(void)> afterEachMovementOfEgoCar) {

  int numberOfProcessedPathElements = points.size()
      - numberOfUnprocessedPathElements;
  for (int i = 0; i < numberOfProcessedPathElements; i++) {
    driveVehicles();
    drive2PointOfEgoCar(points[i].getFrenet(coordsConverter),
                        afterEachMovementOfEgoCar);
  }

  double secsDriven = numberOfProcessedPathElements * dt;
  return secsDriven;
}

void Simulator::driveVehicles() {
  for (Vehicle& vehicle : vehicles) {
    driveVehicle(vehicle);
  }
}

void Simulator::driveVehicle(Vehicle& vehicle) {
  const Frenet vel_frenet = vehicle.getVel_frenet_m_per_s();
  vehicle.setPos_frenet(vehicle.getPos_frenet() + (vel_frenet * dt));
// GTEST_COUT<< "vehicle: " << vehicle.getPos_frenet() << endl;
}

void Simulator::assertNoIncidentsHappened() {
  // TODO: The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
  ASSERT_LT(egoCar.speed_mph, 50);
  ASSERT_FALSE(isCollision(egoCar, vehicles))<< "COLLISION between ego car and another vehicle:" << endl << egoCar << vehicles;
}

void Simulator::drive2PointOfEgoCar(
    const Frenet& dst, function<void(void)> afterEachMovementOfEgoCar) {
  const Frenet& src = egoCar.getPos_frenet();
  egoCar.speed_mph = meter_per_sec2mph(src.distanceTo(dst) / dt);
  egoCar.setPos_frenet(dst);
  egoCar.yaw_deg = rad2deg((dst - src).getHeading());
// GTEST_COUT<< "egoCar: " << egoCar.getPos_frenet() << endl;

  assertNoIncidentsHappened();
  afterEachMovementOfEgoCar();
}

bool Simulator::isCollision(const EgoCar& egoCar, const Vehicle& vehicle) {
  return egoCar.getPos_cart().distanceTo(vehicle.getPos_cart())
      <= EgoCar::carSize();
}

bool Simulator::isCollision(const EgoCar& egoCar,
                            const vector<Vehicle>& vehicles) {
  return std::any_of(
      vehicles.cbegin(), vehicles.cend(),
      [&](const Vehicle& vehicle) {return isCollision(egoCar, vehicle);});
}

#endif /* TESTS_SIMULATOR_H_ */
