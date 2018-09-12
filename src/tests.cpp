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

constexpr int NO_VALUE = -1;

namespace test {

#define GTEST_COUT std::cerr

const double carRadius = 1.25;
const double carSize = 2 * carRadius;

template<typename T, typename R, typename unop>
vector<R> map2(vector<T> v, unop op) {
  vector<R> result(v.size());
  transform(v.begin(), v.end(), result.begin(), op);
  return result;
}

vector<Frenet> asFrenets(const vector<Point> &points,
                         const MapWaypoints &map_waypoints) {

  return map2<Point, Frenet>(points, [&map_waypoints](const Point &point) {
    return getFrenet(point, 0, map_waypoints);
  });
}

bool isCollision(const EgoCar &egoCar, const Vehicle &vehicle) {
  return egoCar.getPos_cart().distanceTo(vehicle.getPos_cart()) <= carSize;
}

bool isCollision(const EgoCar &egoCar, const vector<Vehicle> &vehicles) {
  return std::any_of(
      vehicles.cbegin(), vehicles.cend(),
      [&egoCar](const Vehicle &vehicle) {return isCollision(egoCar, vehicle);});
}

EgoCar createEgoCar(const Frenet &pos, const MapWaypoints &map_waypoints) {
  EgoCar egoCar;
  egoCar.setPos_frenet(pos, map_waypoints);
  egoCar.yaw_deg = 0;
  egoCar.speed_mph = 0;
  return egoCar;
}

void assert_car_drives_in_middle_of_lane(const Path &path, Lane lane,
                                         const MapWaypoints &map_waypoints) {
  for (const Frenet &frenet : asFrenets(path.points, map_waypoints)) {
    ASSERT_NEAR(2 + 4 * lane, frenet.d, 0.001);
  }
}

vector<double> getDistancesAlongRoad(const Path &path,
                                     const MapWaypoints &map_waypoints) {

  return map2<Frenet, double>(asFrenets(path.points, map_waypoints),
                              [](const Frenet &frenet) {return frenet.s;});
}

void assert_car_drives_straight_ahead(const Path &path,
                                      const MapWaypoints &map_waypoints) {
  vector<double> distancesAlongRoad = getDistancesAlongRoad(path,
                                                            map_waypoints);
  ASSERT_TRUE(
      std::is_sorted(distancesAlongRoad.begin(), distancesAlongRoad.end()));
}

void drive2PointOfEgoCar(const Point &dst, EgoCar &egoCar, double dt,
                         const MapWaypoints &map_waypoints,
                         const vector<Vehicle> &vehicles,
                         const function<void(void)>& check) {

  const Point &src = egoCar.getPos_cart();
  egoCar.speed_mph = meter_per_sec2mph(src.distanceTo(dst) / dt);
  egoCar.setPos_cart(dst, map_waypoints);
  egoCar.yaw_deg = rad2deg((dst - src).getHeading());
  // GTEST_COUT<< "egoCar: " << egoCar.getPos_frenet();

  ASSERT_FALSE(isCollision(egoCar, vehicles))<< "COLLISION:" << endl << egoCar << vehicles[0];
  check();
}

void driveVehicle(Vehicle &vehicle, double dt,
                  const MapWaypoints &map_waypoints) {
  const Frenet vel_frenet = vehicle.getVel_frenet_m_per_s(map_waypoints);
  vehicle.setPos_frenet(vehicle.getPos_frenet() + (vel_frenet * dt),
                        map_waypoints);
  // GTEST_COUT<< "vehicle: " << vehicle.getPos_frenet();
}

void driveVehicles(vector<Vehicle> &vehicles, double dt,
                   const MapWaypoints &map_waypoints) {
  for (Vehicle &vehicle : vehicles) {
    driveVehicle(vehicle, dt, map_waypoints);
  }
}

double drive2PointsOfEgoCarAndDriveVehicles(const vector<Point>& points,
                                            int numberOfUnprocessedPathElements,
                                            double dt,
                                            const MapWaypoints& map_waypoints,
                                            const function<void(void)>& check,
                                            EgoCar& egoCar,
                                            vector<Vehicle> &vehicles) {

  int numberOfProcessedPathElements = points.size()
      - numberOfUnprocessedPathElements;
  for (int i = 0; i < numberOfProcessedPathElements; i++) {
    driveVehicles(vehicles, dt, map_waypoints);
    drive2PointOfEgoCar(points[i], egoCar, dt, map_waypoints, vehicles, check);
  }

  double secsDriven = numberOfProcessedPathElements * dt;
  return secsDriven;
}

void updatePreviousData(const vector<Point>& points,
                        int numberOfUnprocessedPathElements, const Path& path,
                        const MapWaypoints& map_waypoints,
                        PreviousData& previousData, const EgoCar& egoCar) {
  previousData.previous_path.points.clear();
  for (int i = points.size() - numberOfUnprocessedPathElements;
      i < points.size(); i++) {
    previousData.previous_path.points.push_back(path.points[i]);
  }
  previousData.end_path = getFrenet(
      points[points.size() - numberOfUnprocessedPathElements - 1],
      deg2rad(egoCar.yaw_deg), map_waypoints);
}

bool oneRoundDriven(const EgoCar &egoCar) {
  return egoCar.getPos_frenet().s > 6900;
}

double driveEgoCarAndVehicles(ReferencePoint &refPoint, Lane &lane,
                              const MapWaypoints &map_waypoints, EgoCar &egoCar,
                              PreviousData &previousData,
                              vector<Vehicle> &vehicles, double dt,
                              const function<void(void)> &check) {

  Path path = createPath(refPoint, lane, map_waypoints, egoCar, previousData,
                         vehicles, dt);
  int numberOfUnprocessedPathElements = 10;
  double secsDriven = drive2PointsOfEgoCarAndDriveVehicles(
      path.points, numberOfUnprocessedPathElements, dt, map_waypoints, check,
      egoCar, vehicles);
  updatePreviousData(path.points, numberOfUnprocessedPathElements, path,
                     map_waypoints, previousData, egoCar);
  return secsDriven;
}

void drive(ReferencePoint &refPoint, Lane &lane,
           const MapWaypoints &map_waypoints, EgoCar &egoCar,
           PreviousData &previousData, vector<Vehicle> &vehicles, double dt,
           int minSecs2Drive, const function<void(void)> &check) {

  double secsDriven = 0;
  while ((secsDriven <= minSecs2Drive || minSecs2Drive == NO_VALUE)
      && !oneRoundDriven(egoCar)) {
    secsDriven += driveEgoCarAndVehicles(refPoint, lane, map_waypoints, egoCar,
                                         previousData, vehicles, dt, check);
  }
}

Vehicle createVehicle(int id, const Frenet &pos, const Frenet &vel_m_per_sec,
                      const MapWaypoints &map_waypoints) {
  Vehicle vehicle;
  vehicle.id = id;
  vehicle.setPos_frenet(pos, map_waypoints);
  vehicle.setVel_frenet_m_per_s(vel_m_per_sec, map_waypoints);
  return vehicle;
}

bool hasBeenInLane(const vector<double> &ds, Lane lane) {
  return std::any_of(ds.cbegin(), ds.cend(),
                     [lane](double d) {return isInLane(d, lane);});
}

std::vector<bool>::iterator getEgoCarJustOvertakesVehicleIterator(
    vector<bool> &overtakens) {

  auto it = find(begin(overtakens), end(overtakens), true);
  return begin(overtakens) + (it - begin(overtakens));
}

bool staysOvertaken(vector<bool>::const_iterator egoCarJustOvertakesVehicle,
                    const vector<bool> &overtakens) {
  return all_of(egoCarJustOvertakesVehicle, end(overtakens),
                [](bool overtaken) {return overtaken;});
}

}

TEST(PathPlanningTest, should_drive_in_same_lane) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  Lane lane = Lane::MIDDLE;
  Frenet pos = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(pos, map_waypoints);

  PreviousData previousData;
  vector<Vehicle> vehicles;

  double dt = 0.02;

// WHEN
  Path path = createPath(refPoint, lane, map_waypoints, egoCar, previousData,
                         vehicles, dt);

// THEN
  test::assert_car_drives_in_middle_of_lane(path, Lane::MIDDLE, map_waypoints);
  test::assert_car_drives_straight_ahead(path, map_waypoints);
}

TEST(PathPlanningTest, should_drive_with_max_50_mph) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  Lane lane = Lane::MIDDLE;
  Frenet pos = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(pos, map_waypoints);

  PreviousData previousData;
  vector<Vehicle> vehicles;

  double dt = 0.02;

// WHEN
  test::drive(
      refPoint, lane, map_waypoints, egoCar, previousData, vehicles, dt,
      NO_VALUE, [&egoCar]() {
        ASSERT_LT(egoCar.speed_mph, 50);
        ASSERT_NEAR(2 + 4 * Lane::MIDDLE, egoCar.getPos_frenet().d, 0.1);});

// THEN
}

TEST(PathPlanningTest, should_collide) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(Lane::MIDDLE) };
  EgoCar egoCar = test::createEgoCar(posCar, map_waypoints);
  Vehicle vehicle = test::createVehicle(
      0, posCar + Frenet { test::carRadius / 2, 0 }, Frenet { 0, 0 },
      map_waypoints);

// WHEN

// THEN
  ASSERT_TRUE(test::isCollision(egoCar, vehicle));
}

TEST(PathPlanningTest, should_not_collide) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  double dt = 0.02;
  PreviousData previousData;
  Lane lane = Lane::MIDDLE;
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(posCar, map_waypoints);
  Vehicle vehicle = test::createVehicle(0, posCar + Frenet { 10 * test::carSize,
      0 },
                                        Frenet { 5, 0 }, map_waypoints);
  vector<Vehicle> vehicles = { vehicle };

// WHEN
  test::drive(refPoint, lane, map_waypoints, egoCar, previousData, vehicles, dt,
              NO_VALUE, [&egoCar, &vehicles]() {
                ASSERT_FALSE(test::isCollision(egoCar, vehicles));});

// THEN
}

TEST(PathPlanningTest, should_overtake_vehicle) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  double dt = 0.02;
  PreviousData previousData;
  Lane lane = Lane::MIDDLE;
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(posCar, map_waypoints);
  Vehicle vehicle = test::createVehicle(0, posCar + Frenet { 35, 0 }, Frenet {
                                            mph2meter_per_sec(5), 0 },
                                        map_waypoints);
  vector<Vehicle> vehicles = { vehicle };

// WHEN
  vector<bool> overtakens;
  test::drive(
      refPoint,
      lane,
      map_waypoints,
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

TEST(PathPlanningTest, should_overtake_vehicle2) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  double dt = 0.02;
  PreviousData previousData;
  Lane lane = Lane::MIDDLE;
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(posCar, map_waypoints);
  Vehicle vehicle2Overtake = test::createVehicle(0, posCar + Frenet { 35, 0 },
                                                 Frenet { mph2meter_per_sec(5),
                                                     0 },
                                                 map_waypoints);
  Vehicle vehicleInLeftLane = test::createVehicle(
      1, Frenet { posCar.s + 35, getMiddleOfLane(Lane::LEFT) }, Frenet {
          mph2meter_per_sec(5), 0 },
      map_waypoints);
  vector<Vehicle> vehicles = { vehicle2Overtake, vehicleInLeftLane };

// WHEN
  vector<bool> overtakens;
  test::drive(
      refPoint,
      lane,
      map_waypoints,
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
