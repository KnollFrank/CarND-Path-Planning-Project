#include "gtest/gtest.h"
#include "../pathPlanner.h"
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include "../json.hpp"
#include <tuple>
#include "../spline.h"
#include "../path.h"
#include "simulator.h"


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

  test::Simulator simulator(
      refPoint, lane, coordsConverter, egoCar, previousData, vehicles, dt,
      NO_VALUE, [&egoCar]() {
        ASSERT_LT(egoCar.speed_mph, 50);
        ASSERT_NEAR(2 + 4 * Lane::MIDDLE, egoCar.getPos_frenet().d, 0.9);});

// WHEN
  simulator.drive();

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

  test::Simulator simulator(
      refPoint, lane, coordsConverter, egoCar, previousData, vehicles, dt,
      NO_VALUE, [&egoCar, &vehicles]() {
        ASSERT_FALSE(test::isCollision(egoCar, vehicles));});

// WHEN
  simulator.drive();

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

  vector<bool> overtakens;
  test::Simulator simulator(
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

// WHEN
  simulator.drive();

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

  vector<bool> overtakens;
  test::Simulator simulator(
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

// WHEN
  simulator.drive();

  // THEN
  auto egoCarJustOvertakesVehicle = test::getEgoCarJustOvertakesVehicleIterator(
      overtakens);
  ASSERT_NE(egoCarJustOvertakesVehicle, end(overtakens))<< "egoCar should overtake vehicle";
  ASSERT_TRUE(test::staysOvertaken(egoCarJustOvertakesVehicle, overtakens))<< "egoCar should stay ahead of vehicle";
}
