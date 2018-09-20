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

class PathPlannerTest : public ::testing::Test {

 protected:
  void SetUp() override {
    mapWaypoints = read_map_waypoints();
    coordsConverter = new CoordsConverter(mapWaypoints);
    refPoint.vel_mph = 0;
  }

  void TearDown() override {
    delete coordsConverter;
  }

  Simulator createSimulator(Lane& lane, EgoCar& egoCar,
                            vector<Vehicle>& vehicles, int minSecs2Drive) {
    return Simulator(refPoint, lane, *coordsConverter, egoCar, previousData,
                     vehicles, 0.02, minSecs2Drive);
  }

  EgoCar createEgoCar(const Frenet& pos) {
    EgoCar egoCar(*coordsConverter);
    egoCar.setPos_frenet(pos);
    egoCar.yaw_deg = 0;
    egoCar.speed_mph = 0;
    return egoCar;
  }

  Vehicle createVehicle(int id, const Frenet& pos,
                        const Frenet& vel_m_per_sec) {
    Vehicle vehicle(*coordsConverter);
    vehicle.id = id;
    vehicle.setPos_frenet(pos);
    vehicle.setVel_frenet_m_per_s(vel_m_per_sec);
    return vehicle;
  }

  vector<Frenet> asFrenets(const vector<Point>& points) {

    return map2<Point, Frenet>(points, [&](const Point& point) {
      return coordsConverter->getFrenet(point);
    });
  }

  void assert_car_drives_in_middle_of_lane(const Path& path, Lane lane) {

    for (const Frenet& frenet : asFrenets(path.points)) {
      ASSERT_NEAR(2 + 4 * lane, frenet.d, 0.001);
    }
  }

  vector<double> getDistancesAlongRoad(const Path& path) {

    return map2<Frenet, double>(asFrenets(path.points),
                                [](const Frenet& frenet) {return frenet.s;});
  }

  void assert_car_drives_straight_ahead(const Path& path) {
    vector<double> distancesAlongRoad = getDistancesAlongRoad(path);
    ASSERT_TRUE(
        std::is_sorted(distancesAlongRoad.begin(), distancesAlongRoad.end()));
  }

  MapWaypoints mapWaypoints;
  CoordsConverter* coordsConverter;
  ReferencePoint refPoint;
  PreviousData previousData;
};

TEST_F(PathPlannerTest, should_drive_in_same_lane) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) });
  vector<Vehicle> vehicles;

  PathPlanner pathPlanner(*coordsConverter, refPoint, lane, 0.02);

// WHEN
  Path path = pathPlanner.createPath(egoCar, previousData, vehicles);

// THEN
  assert_car_drives_in_middle_of_lane(path, Lane::MIDDLE);
  assert_car_drives_straight_ahead(path);
}

TEST_F(PathPlannerTest, should_drive_with_max_50_mph) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) });
  vector<Vehicle> vehicles;

  Simulator simulator = createSimulator(lane, egoCar, vehicles, NO_VALUE);

// WHEN
  simulator.drive([&egoCar]() {
    ASSERT_LT(egoCar.speed_mph, 50);
    ASSERT_NEAR(2 + 4 * Lane::MIDDLE, egoCar.getPos_frenet().d, 0.9);});

// THEN
}

TEST_F(PathPlannerTest, should_collide) {
// GIVEN
  EgoCar egoCar = createEgoCar(
      Frenet { 124.8336, getMiddleOfLane(Lane::MIDDLE) });
  Vehicle vehicle = createVehicle(
      0, egoCar.getPos_frenet() + Frenet { EgoCar::carRadius() / 2, 0 },
      Frenet { 0, 0 });

// WHEN

// THEN
  ASSERT_TRUE(Simulator::isCollision(egoCar, vehicle));
}

TEST_F(PathPlannerTest, should_not_collide) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) });
  Vehicle vehicle = createVehicle(
      0, egoCar.getPos_frenet() + Frenet { 10 * EgoCar::carSize(), 0 }, Frenet {
          5, 0 });
  vector<Vehicle> vehicles = { vehicle };

  Simulator simulator = createSimulator(lane, egoCar, vehicles, NO_VALUE);

// WHEN
  simulator.drive([&]() {
    ASSERT_FALSE(Simulator::isCollision(egoCar, vehicles));});

// THEN
}

TEST_F(PathPlannerTest, should_overtake_vehicle) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) });
  Vehicle vehicle = createVehicle(0, egoCar.getPos_frenet() + Frenet { 35, 0 },
                                  Frenet { mph2meter_per_sec(5), 0 });
  vector<Vehicle> vehicles = { vehicle };

  Simulator simulator = createSimulator(lane, egoCar, vehicles, 60);

// WHEN
  vector<bool> overtakens;
  simulator.drive([&]() {
    bool overtaken = egoCar.getPos_frenet().s > vehicles[0].getPos_frenet().s;
    overtakens.push_back(overtaken);});

// THEN
  auto egoCarJustOvertakesVehicle = getEgoCarJustOvertakesVehicleIterator(
      overtakens);
  ASSERT_NE(egoCarJustOvertakesVehicle, end(overtakens))<< "egoCar should overtake vehicle";
  ASSERT_TRUE(staysOvertaken(egoCarJustOvertakesVehicle, overtakens))<< "egoCar should stay ahead of vehicle";
}

TEST_F(PathPlannerTest, should_overtake_vehicle2) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) });
  Vehicle vehicle2Overtake = createVehicle(0, egoCar.getPos_frenet() + Frenet {
      35, 0 },
                                           Frenet { mph2meter_per_sec(5), 0 });
  Vehicle vehicleInLeftLane = createVehicle(
      1, Frenet { egoCar.getPos_frenet().s + 35, getMiddleOfLane(Lane::LEFT) },
      Frenet { mph2meter_per_sec(5), 0 });
  vector<Vehicle> vehicles { vehicle2Overtake, vehicleInLeftLane };

  Simulator simulator = createSimulator(lane, egoCar, vehicles, 60);

// WHEN
  vector<bool> overtakens;
  simulator.drive([&]() {
    bool overtaken = egoCar.getPos_frenet().s > vehicles[0].getPos_frenet().s;
    overtakens.push_back(overtaken);});

  // THEN
  auto egoCarJustOvertakesVehicle = getEgoCarJustOvertakesVehicleIterator(
      overtakens);
  ASSERT_NE(egoCarJustOvertakesVehicle, end(overtakens))<< "egoCar should overtake vehicle";
  ASSERT_TRUE(staysOvertaken(egoCarJustOvertakesVehicle, overtakens))<< "egoCar should stay ahead of vehicle";
}
