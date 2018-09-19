#include "gtest/gtest.h"
#include "pathPlanner.h"
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
#include "path.h"

constexpr int NO_VALUE = -1;

namespace test {

#define GTEST_COUT std::cerr

const double carRadius = 1.25;
const double carSize = 2 * carRadius;

vector<Frenet> asFrenets(const vector<Point>& points,
                         const CoordsConverter& coordsConverter) {

  return map2<Point, Frenet>(points, [&coordsConverter](const Point& point) {
    return coordsConverter.getFrenet(point);
  });
}

bool isCollision(const EgoCar& egoCar, const Vehicle& vehicle) {
  return egoCar.getPos_cart().distanceTo(vehicle.getPos_cart()) <= carSize;
}

bool isCollision(const EgoCar& egoCar, const vector<Vehicle>& vehicles) {
  return std::any_of(
      vehicles.cbegin(), vehicles.cend(),
      [&egoCar](const Vehicle& vehicle) {return isCollision(egoCar, vehicle);});
}

EgoCar createEgoCar(const Frenet& pos, const CoordsConverter& coordsConverter) {
  EgoCar egoCar(coordsConverter);
  egoCar.setPos_frenet(pos);
  egoCar.yaw_deg = 0;
  egoCar.speed_mph = 0;
  return egoCar;
}

void assert_car_drives_in_middle_of_lane(
    const Path& path, Lane lane, const CoordsConverter& coordsConverter) {

  for (const Frenet& frenet : asFrenets(path.points, coordsConverter)) {
    ASSERT_NEAR(2 + 4 * lane, frenet.d, 0.001);
  }
}

vector<double> getDistancesAlongRoad(const Path& path,
                                     const CoordsConverter& coordsConverter) {

  return map2<Frenet, double>(asFrenets(path.points, coordsConverter),
                              [](const Frenet& frenet) {return frenet.s;});
}

void assert_car_drives_straight_ahead(const Path& path,
                                      const CoordsConverter& coordsConverter) {
  vector<double> distancesAlongRoad = getDistancesAlongRoad(path,
                                                            coordsConverter);
  ASSERT_TRUE(
      std::is_sorted(distancesAlongRoad.begin(), distancesAlongRoad.end()));
}

void drive2PointOfEgoCar(const Point& dst, EgoCar& egoCar, double dt,
                         const vector<Vehicle>& vehicles,
                         const function<void(void)>& check) {

  const Point& src = egoCar.getPos_cart();
  egoCar.speed_mph = meter_per_sec2mph(src.distanceTo(dst) / dt);
  egoCar.setPos_cart(dst);
  egoCar.yaw_deg = rad2deg((dst - src).getHeading());
  // GTEST_COUT<< "egoCar: " << egoCar.getPos_frenet() << endl;

  ASSERT_FALSE(isCollision(egoCar, vehicles))<< "COLLISION:" << endl << egoCar << vehicles[0];
  check();
}

void driveVehicle(Vehicle& vehicle, double dt) {
  const Frenet vel_frenet = vehicle.getVel_frenet_m_per_s();
  vehicle.setPos_frenet(vehicle.getPos_frenet() + (vel_frenet * dt));
  // GTEST_COUT<< "vehicle: " << vehicle.getPos_frenet() << endl;
}

void driveVehicles(vector<Vehicle>& vehicles, double dt) {
  for (Vehicle& vehicle : vehicles) {
    driveVehicle(vehicle, dt);
  }
}

double drive2PointsOfEgoCarAndDriveVehicles(const vector<Point>& points,
                                            int numberOfUnprocessedPathElements,
                                            double dt,
                                            const function<void(void)>& check,
                                            EgoCar& egoCar,
                                            vector<Vehicle>& vehicles) {

  int numberOfProcessedPathElements = points.size()
      - numberOfUnprocessedPathElements;
  for (int i = 0; i < numberOfProcessedPathElements; i++) {
    driveVehicles(vehicles, dt);
    drive2PointOfEgoCar(points[i], egoCar, dt, vehicles, check);
  }

  double secsDriven = numberOfProcessedPathElements * dt;
  return secsDriven;
}

void updatePreviousData(const vector<Point>& points,
                        int numberOfUnprocessedPathElements, const Path& path,
                        const CoordsConverter& coordsConverter,
                        PreviousData& previousData, const EgoCar& egoCar) {
  previousData.previous_path.points.clear();
  for (int i = points.size() - numberOfUnprocessedPathElements;
      i < points.size(); i++) {
    previousData.previous_path.points.push_back(path.points[i]);
  }
  previousData.end_path = coordsConverter.getFrenet(
      points[points.size() - numberOfUnprocessedPathElements - 1]);
}

bool oneRoundDriven(const EgoCar& egoCar) {
  return egoCar.getPos_frenet().s > 6900;
}

double driveEgoCarAndVehicles(ReferencePoint& refPoint, Lane& lane,
                              const CoordsConverter& coordsConverter,
                              EgoCar& egoCar, PreviousData& previousData,
                              vector<Vehicle>& vehicles, double dt,
                              const function<void(void)>& check) {

  PathPlanner pathPlanner(coordsConverter, refPoint, lane, dt);
  Path path = pathPlanner.createPath(egoCar, previousData, vehicles);
  int numberOfUnprocessedPathElements = 10;
  double secsDriven = drive2PointsOfEgoCarAndDriveVehicles(
      path.points, numberOfUnprocessedPathElements, dt, check, egoCar,
      vehicles);
  updatePreviousData(path.points, numberOfUnprocessedPathElements, path,
                     coordsConverter, previousData, egoCar);
  return secsDriven;
}

// TODO: make drive method a class
void drive(ReferencePoint& refPoint, Lane& lane,
           const CoordsConverter& coordsConverter, EgoCar& egoCar,
           PreviousData& previousData, vector<Vehicle>& vehicles, double dt,
           int minSecs2Drive, const function<void(void)>& check) {

  double secsDriven = 0;
  while ((secsDriven <= minSecs2Drive || minSecs2Drive == NO_VALUE)
      && !oneRoundDriven(egoCar)) {
    secsDriven += driveEgoCarAndVehicles(refPoint, lane, coordsConverter,
                                         egoCar, previousData, vehicles, dt,
                                         check);
  }
}

Vehicle createVehicle(int id, const Frenet& pos, const Frenet& vel_m_per_sec,
                      const CoordsConverter& coordsConverter) {
  Vehicle vehicle(coordsConverter);
  vehicle.id = id;
  vehicle.setPos_frenet(pos);
  vehicle.setVel_frenet_m_per_s(vel_m_per_sec);
  return vehicle;
}

bool hasBeenInLane(const vector<double>& ds, Lane lane) {
  return std::any_of(ds.cbegin(), ds.cend(),
                     [lane](double d) {return isInLane(d, lane);});
}

std::vector<bool>::iterator getEgoCarJustOvertakesVehicleIterator(
    vector<bool>& overtakens) {

  auto it = find(begin(overtakens), end(overtakens), true);
  return begin(overtakens) + (it - begin(overtakens));
}

bool staysOvertaken(vector<bool>::const_iterator egoCarJustOvertakesVehicle,
                    const vector<bool>& overtakens) {
  return all_of(egoCarJustOvertakesVehicle, end(overtakens),
                [](bool overtaken) {return overtaken;});
}

}

TEST(PathPlannerTest, should_drive_in_same_lane) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  Lane lane = Lane::MIDDLE;
  Frenet pos = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(pos, coordsConverter);

  PreviousData previousData;
  vector<Vehicle> vehicles;

  double dt = 0.02;

  PathPlanner pathPlanner(coordsConverter, refPoint, lane, dt);

// WHEN
  Path path = pathPlanner.createPath(egoCar, previousData, vehicles);

// THEN
  test::assert_car_drives_in_middle_of_lane(path, Lane::MIDDLE,
                                            coordsConverter);
  test::assert_car_drives_straight_ahead(path, coordsConverter);
}

TEST(PathPlannerTest, should_drive_with_max_50_mph) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  Lane lane = Lane::MIDDLE;
  Frenet pos = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(pos, coordsConverter);

  PreviousData previousData;
  vector<Vehicle> vehicles;

  double dt = 0.02;

// WHEN
  test::drive(
      refPoint, lane, coordsConverter, egoCar, previousData, vehicles, dt,
      NO_VALUE, [&egoCar]() {
        ASSERT_LT(egoCar.speed_mph, 50);
        ASSERT_NEAR(2 + 4 * Lane::MIDDLE, egoCar.getPos_frenet().d, 0.9);});

// THEN
}

TEST(PathPlannerTest, should_collide) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(Lane::MIDDLE) };
  EgoCar egoCar = test::createEgoCar(posCar, coordsConverter);
  Vehicle vehicle = test::createVehicle(
      0, posCar + Frenet { test::carRadius / 2, 0 }, Frenet { 0, 0 },
      coordsConverter);

// WHEN

// THEN
  ASSERT_TRUE(test::isCollision(egoCar, vehicle));
}

TEST(PathPlannerTest, should_not_collide) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  double dt = 0.02;
  PreviousData previousData;
  Lane lane = Lane::MIDDLE;
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(posCar, coordsConverter);
  Vehicle vehicle = test::createVehicle(0, posCar + Frenet { 10 * test::carSize,
      0 },
                                        Frenet { 5, 0 }, coordsConverter);
  vector<Vehicle> vehicles = { vehicle };

// WHEN
  test::drive(refPoint, lane, coordsConverter, egoCar, previousData, vehicles,
              dt, NO_VALUE, [&egoCar, &vehicles]() {
                ASSERT_FALSE(test::isCollision(egoCar, vehicles));});

// THEN
}

TEST(PathPlannerTest, should_overtake_vehicle) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  double dt = 0.02;
  PreviousData previousData;
  Lane lane = Lane::MIDDLE;
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(posCar, coordsConverter);
  Vehicle vehicle = test::createVehicle(0, posCar + Frenet { 35, 0 }, Frenet {
                                            mph2meter_per_sec(5), 0 },
                                        coordsConverter);
  vector<Vehicle> vehicles = { vehicle };

// WHEN
  vector<bool> overtakens;
  test::drive(
      refPoint,
      lane,
      coordsConverter,
      egoCar,
      previousData,
      vehicles,
      dt,
      60,
      [&egoCar, &vehicles, &overtakens]() {
        bool overtaken = egoCar.getPos_frenet().s > vehicles[0].getPos_frenet().s;
        overtakens.push_back(overtaken);});

  // THEN
  auto egoCarJustOvertakesVehicle = test::getEgoCarJustOvertakesVehicleIterator(
      overtakens);
  ASSERT_NE(egoCarJustOvertakesVehicle, end(overtakens))<< "egoCar should overtake vehicle";
  ASSERT_TRUE(test::staysOvertaken(egoCarJustOvertakesVehicle, overtakens))<< "egoCar should stay ahead of vehicle";
}

TEST(PathPlannerTest, should_overtake_vehicle2) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  double dt = 0.02;
  PreviousData previousData;
  Lane lane = Lane::MIDDLE;
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(posCar, coordsConverter);
  Vehicle vehicle2Overtake = test::createVehicle(0, posCar + Frenet { 35, 0 },
                                                 Frenet { mph2meter_per_sec(5),
                                                     0 },
                                                 coordsConverter);
  Vehicle vehicleInLeftLane = test::createVehicle(
      1, Frenet { posCar.s + 35, getMiddleOfLane(Lane::LEFT) }, Frenet {
          mph2meter_per_sec(5), 0 },
      coordsConverter);
  vector<Vehicle> vehicles = { vehicle2Overtake, vehicleInLeftLane };

// WHEN
  vector<bool> overtakens;
  test::drive(
      refPoint,
      lane,
      coordsConverter,
      egoCar,
      previousData,
      vehicles,
      dt,
      60,
      [&egoCar, &vehicles, &overtakens]() {
        bool overtaken = egoCar.getPos_frenet().s > vehicles[0].getPos_frenet().s;
        overtakens.push_back(overtaken);});

  // THEN
  auto egoCarJustOvertakesVehicle = test::getEgoCarJustOvertakesVehicleIterator(
      overtakens);
  ASSERT_NE(egoCarJustOvertakesVehicle, end(overtakens))<< "egoCar should overtake vehicle";
  ASSERT_TRUE(test::staysOvertaken(egoCarJustOvertakesVehicle, overtakens))<< "egoCar should stay ahead of vehicle";
}
