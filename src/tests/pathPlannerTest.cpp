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
  EgoCar egoCar = createEgoCar(pos, coordsConverter);

  PreviousData previousData;
  vector<Vehicle> vehicles;

  PathPlanner pathPlanner(coordsConverter, refPoint, lane, 0.02);

// WHEN
  Path path = pathPlanner.createPath(egoCar, previousData, vehicles);

// THEN
  assert_car_drives_in_middle_of_lane(path, Lane::MIDDLE, coordsConverter);
  assert_car_drives_straight_ahead(path, coordsConverter);
}

TEST(PathPlannerTest, should_drive_with_max_50_mph) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  Lane lane = Lane::MIDDLE;
  Frenet pos = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = createEgoCar(pos, coordsConverter);

  PreviousData previousData;
  vector<Vehicle> vehicles;

  Simulator simulator(refPoint, lane, coordsConverter, egoCar, previousData,
                      vehicles, 0.02, NO_VALUE);

// WHEN
  simulator.drive([&egoCar]() {
    ASSERT_LT(egoCar.speed_mph, 50);
    ASSERT_NEAR(2 + 4 * Lane::MIDDLE, egoCar.getPos_frenet().d, 0.9);});

// THEN
}

TEST(PathPlannerTest, should_collide) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(Lane::MIDDLE) };
  EgoCar egoCar = createEgoCar(posCar, coordsConverter);
  Vehicle vehicle = createVehicle(0, posCar + Frenet { carRadius / 2, 0 },
                                  Frenet { 0, 0 }, coordsConverter);

// WHEN

// THEN
  ASSERT_TRUE(isCollision(egoCar, vehicle));
}

TEST(PathPlannerTest, should_not_collide) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  PreviousData previousData;
  Lane lane = Lane::MIDDLE;
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = createEgoCar(posCar, coordsConverter);
  Vehicle vehicle = createVehicle(0, posCar + Frenet { 10 * carSize, 0 },
                                  Frenet { 5, 0 }, coordsConverter);
  vector<Vehicle> vehicles = { vehicle };

  Simulator simulator(refPoint, lane, coordsConverter, egoCar, previousData,
                      vehicles, 0.02, NO_VALUE);

// WHEN
  simulator.drive([&egoCar, &vehicles]() {
    ASSERT_FALSE(isCollision(egoCar, vehicles));});

// THEN
}

TEST(PathPlannerTest, should_overtake_vehicle) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  PreviousData previousData;
  Lane lane = Lane::MIDDLE;
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = createEgoCar(posCar, coordsConverter);
  Vehicle vehicle = createVehicle(0, posCar + Frenet { 35, 0 }, Frenet {
                                      mph2meter_per_sec(5), 0 },
                                  coordsConverter);
  vector<Vehicle> vehicles = { vehicle };

  Simulator simulator(refPoint, lane, coordsConverter, egoCar, previousData,
                      vehicles, 0.02, 60);

// WHEN
  vector<bool> overtakens;
  simulator.drive([&egoCar, &vehicles, &overtakens]() {
    bool overtaken = egoCar.getPos_frenet().s > vehicles[0].getPos_frenet().s;
    overtakens.push_back(overtaken);});

// THEN
  auto egoCarJustOvertakesVehicle = getEgoCarJustOvertakesVehicleIterator(
      overtakens);
  ASSERT_NE(egoCarJustOvertakesVehicle, end(overtakens))<< "egoCar should overtake vehicle";
  ASSERT_TRUE(staysOvertaken(egoCarJustOvertakesVehicle, overtakens))<< "egoCar should stay ahead of vehicle";
}

TEST(PathPlannerTest, should_overtake_vehicle2) {
// GIVEN
  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  PreviousData previousData;
  Lane lane = Lane::MIDDLE;
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = createEgoCar(posCar, coordsConverter);
  Vehicle vehicle2Overtake = createVehicle(0, posCar + Frenet { 35, 0 },
                                           Frenet { mph2meter_per_sec(5), 0 },
                                           coordsConverter);
  Vehicle vehicleInLeftLane = createVehicle(1, Frenet { posCar.s + 35,
                                                getMiddleOfLane(Lane::LEFT) },
                                            Frenet { mph2meter_per_sec(5), 0 },
                                            coordsConverter);
  vector<Vehicle> vehicles = { vehicle2Overtake, vehicleInLeftLane };

  Simulator simulator(refPoint, lane, coordsConverter, egoCar, previousData,
                      vehicles, 0.02, 60);

// WHEN
  vector<bool> overtakens;
  simulator.drive([&egoCar, &vehicles, &overtakens]() {
    bool overtaken = egoCar.getPos_frenet().s > vehicles[0].getPos_frenet().s;
    overtakens.push_back(overtaken);});

  // THEN
  auto egoCarJustOvertakesVehicle = getEgoCarJustOvertakesVehicleIterator(
      overtakens);
  ASSERT_NE(egoCarJustOvertakesVehicle, end(overtakens))<< "egoCar should overtake vehicle";
  ASSERT_TRUE(staysOvertaken(egoCarJustOvertakesVehicle, overtakens))<< "egoCar should stay ahead of vehicle";
}
