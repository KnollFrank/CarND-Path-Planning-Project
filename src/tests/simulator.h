#ifndef TESTS_SIMULATOR_H_
#define TESTS_SIMULATOR_H_

#include <boost/circular_buffer/base.hpp>
#include <coords/cart.h>
#include <coords/coordsConverter.h>
#include <coords/frenet.h>
#include <coords/frenetCart.h>
#include <egoCar.h>
#include <funs.h>
#include <gtest/gtest.h>
#include <gtest/gtest-message.h>
#include <lane.h>
#include <parametricSpline.h>
#include <path.h>
#include <pathPlanner.h>
#include <previousData.h>
#include <rectangle.h>
#include <referencePoint.h>
#include <tests/gtestHelper.h>
#include <tests/vehicleDriver.h>
#include <vehicle.h>
#include <algorithm>
#include <iostream>
#include <vector>

using namespace std;
using namespace std::experimental;

#define GTEST_COUT std::cerr

class Simulator {
 public:
  Simulator(ReferencePoint& refPoint, Lane& lane,
            const CoordsConverter& coordsConverter, EgoCar& egoCar,
            PreviousData& previousData, vector<Vehicle>& vehicles, double dt,
            std::experimental::optional<int> minSecs2Drive,
            VehicleDriver* vehicleDriver, const double speed_limit_mph);
  ~Simulator();
  void run(function<void(void)> afterEachMovementOfEgoCar);
  static bool isCollision(const EgoCar& egoCar, const Vehicle& vehicle);
  static bool isCollision(const EgoCar& egoCar,
                          const vector<Vehicle>& vehicles);
  static std::experimental::optional<Vehicle> getCollidingVehicle(
      const EgoCar& egoCar, const vector<Vehicle>& vehicles);
  static bool oneRoundDriven(const EgoCar& egoCar);

 private:
  double driveEgoCarAndVehicles(function<void(void)> afterEachMovementOfEgoCar);
  double drive2PointsOfEgoCarAndDriveVehicles(
      const vector<FrenetCart>& points, int numberOfUnprocessedPathElements,
      function<void(void)> afterEachMovementOfEgoCar);
  void driveVehicles();
  void driveVehicle(Vehicle& vehicle);
  void drive2PointOfEgoCar(const FrenetCart& dst,
                           function<void(void)> afterEachMovementOfEgoCar);
  void updatePreviousData(const vector<FrenetCart>& points,
                          int numberOfUnprocessedPathElements,
                          const Path& path);
  bool oneRoundDriven();
  void assertNoIncidentsHappened(double dt);
  double getYawDeg(const Frenet& src, const Frenet& dst);

  ReferencePoint& refPoint;
  Lane& lane;
  const CoordsConverter& coordsConverter;
  EgoCar& egoCar;
  PreviousData& previousData;
  vector<Vehicle>& vehicles;
  double dt;
  std::experimental::optional<int> minSecs2Drive;
  function<void(void)> afterEachMovementOfEgoCar;
  VehicleDriver* vehicleDriver;
 public:
  const double speed_limit_mph;
};

Simulator::Simulator(ReferencePoint& _refPoint, Lane& _lane,
                     const CoordsConverter& _coordsConverter, EgoCar& _egoCar,
                     PreviousData& _previousData, vector<Vehicle>& _vehicles,
                     double _dt,
                     std::experimental::optional<int> _minSecs2Drive,
                     VehicleDriver* _vehicleDriver,
                     const double _speed_limit_mph)
    : refPoint(_refPoint),
      lane(_lane),
      coordsConverter(_coordsConverter),
      egoCar(_egoCar),
      previousData(_previousData),
      vehicles(_vehicles),
      dt(_dt),
      minSecs2Drive(_minSecs2Drive),
      vehicleDriver(_vehicleDriver),
      speed_limit_mph(_speed_limit_mph) {
}

Simulator::~Simulator() {
  delete vehicleDriver;
}

void Simulator::run(function<void(void)> afterEachMovementOfEgoCar) {
  double secsDriven = 0;
  while ((!minSecs2Drive || secsDriven <= minSecs2Drive.value())
      && !oneRoundDriven()) {
    secsDriven += driveEgoCarAndVehicles(afterEachMovementOfEgoCar);
  }
}

bool Simulator::oneRoundDriven(const EgoCar& egoCar) {
  return egoCar.getPos().getFrenet().s > 6900;  // 6947.2427832056264
}

bool Simulator::oneRoundDriven() {
  return oneRoundDriven(egoCar);
}

double Simulator::driveEgoCarAndVehicles(
    function<void(void)> afterEachMovementOfEgoCar) {
  PathPlanner pathPlanner(coordsConverter, refPoint, lane, dt,
                          0.89 * speed_limit_mph, vehicles, egoCar,
                          previousData);
  Path path;
  Lane newLane;
  ReferencePoint refPointNew;
  tie(path, newLane, refPointNew) = pathPlanner.createPath();
  refPoint = refPointNew;
  lane = newLane;
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
  previousData.end_path = previousData.previous_path.points.back();
}

double Simulator::drive2PointsOfEgoCarAndDriveVehicles(
    const vector<FrenetCart>& points, int numberOfUnprocessedPathElements,
    function<void(void)> afterEachMovementOfEgoCar) {

  int numberOfProcessedPathElements = points.size()
      - numberOfUnprocessedPathElements;
  for (int i = 0; i < numberOfProcessedPathElements; i++) {
    driveVehicles();
    drive2PointOfEgoCar(points[i], afterEachMovementOfEgoCar);
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
  vehicleDriver->driveVehicle(vehicle, egoCar, dt);
}

template<typename T>
void print_circular_buffer(string name, const boost::circular_buffer<T>& xs) {
  GTEST_COUT<< name << " = [";
  for (int i = 0; i < xs.size(); i++) {
    GTEST_COUT<< xs[i] << ", " << endl;
  }
  GTEST_COUT<< "]";
}

void Simulator::assertNoIncidentsHappened(double dt) {
  const double acceleration = egoCar.getAcceleration(dt).len();
  if(acceleration > 10) {
    GTEST_COUT << "acceleration = " << acceleration << endl;
    print_circular_buffer("positions", egoCar.positions);
  }
  ASSERT_LE(acceleration, 10)<< egoCar;
  ASSERT_LE(egoCar.getJerk(dt).len(), 10)<< egoCar;
  ASSERT_LE(egoCar.speed_mph, speed_limit_mph)<< egoCar;
  std::experimental::optional<Vehicle> collidingVehicle = getCollidingVehicle(
      egoCar, vehicles);
  ASSERT_FALSE(collidingVehicle)<< "COLLISION between" << endl << egoCar << endl << " and " << endl << *collidingVehicle;
}

double Simulator::getYawDeg(const Frenet& src, const Frenet& dst) {
  Frenet diff = dst.minusCircular(src,
                                  coordsConverter.getSpline()->getLength());
  return rad2deg(diff.getHeading());
}

void Simulator::drive2PointOfEgoCar(
    const FrenetCart& dst, function<void(void)> afterEachMovementOfEgoCar) {
  const FrenetCart& src = egoCar.getPos();
  egoCar.speed_mph = meter_per_sec2mph(
      src.getXY().distanceTo(dst.getXY()) / dt);
  egoCar.setPos(dst);
  egoCar.yaw_deg = getYawDeg(src.getFrenet(), dst.getFrenet());
  GTEST_COUT<< "egoCar: " << egoCar.getPos().getFrenet() << endl;

  assertNoIncidentsHappened(dt);
  afterEachMovementOfEgoCar();
}

bool Simulator::isCollision(const EgoCar& egoCar, const Vehicle& vehicle) {
  return egoCar.getShape().overlaps(vehicle.getShape());
}

bool Simulator::isCollision(const EgoCar& egoCar,
                            const vector<Vehicle>& vehicles) {
  return bool(getCollidingVehicle(egoCar, vehicles));
}

std::experimental::optional<Vehicle> Simulator::getCollidingVehicle(
    const EgoCar& egoCar, const vector<Vehicle>& vehicles) {

  auto collidingVehicleIterator = std::find_if(
      vehicles.cbegin(), vehicles.cend(),
      [&](const Vehicle& vehicle) {return isCollision(egoCar, vehicle);});
  return
      collidingVehicleIterator != vehicles.cend() ?
          std::experimental::make_optional(*collidingVehicleIterator) :
          std::experimental::nullopt;
}

#endif /* TESTS_SIMULATOR_H_ */
