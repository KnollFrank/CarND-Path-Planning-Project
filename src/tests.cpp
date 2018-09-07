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
#include "main.h"
#include "spline.h"

namespace test {

#define GTEST_COUT std::cerr

const double carRadius = 5;

template<typename T, typename R, typename unop>
vector<R> map2(vector<T> v, unop op) {
  vector<R> result(v.size());
  transform(v.begin(), v.end(), result.begin(), op);
  return result;
}

vector<Point> getPoints(const Points &path, const MapWaypoints &map_waypoints) {
  vector<Point> points;
  for (int i = 0; i < path.xs.size(); i++) {
    points.push_back(Point { path.xs[i], path.ys[i] });
  }
  return points;
}

vector<Frenet> asFrenets(const vector<Point> &points,
                         const MapWaypoints &map_waypoints) {

  return map2<Point, Frenet>(points, [&map_waypoints](const Point &point) {
    return getFrenet(point, 0, map_waypoints);
  });
}

bool isCollision(const EgoCar &egoCar, const Vehicle &vehicle) {
  return distance(egoCar.getPos_cart(), vehicle.pos_cart) <= 2 * carRadius;
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
  egoCar.speed = 0;
  return egoCar;
}

void assert_car_drives_in_middle_of_lane(const Points &path, int lane,
                                         const MapWaypoints &map_waypoints) {
  for (const Frenet &frenet : asFrenets(getPoints(path, map_waypoints),
                                        map_waypoints)) {
    ASSERT_NEAR(2 + 4 * lane, frenet.d, 0.001);
  }
}

vector<double> getDistancesAlongRoad(const Points &path,
                                     const MapWaypoints &map_waypoints) {

  return map2<Frenet, double>(
      asFrenets(getPoints(path, map_waypoints), map_waypoints),
      [](const Frenet &frenet) {return frenet.s;});
}

void assert_car_drives_straight_ahead(const Points &path,
                                      const MapWaypoints &map_waypoints) {
  vector<double> distancesAlongRoad = getDistancesAlongRoad(path,
                                                            map_waypoints);
  ASSERT_TRUE(
      std::is_sorted(distancesAlongRoad.begin(), distancesAlongRoad.end()));
}

void check_and_assert_no_collision(const function<void(void)>& check,
                                   const EgoCar &egoCar,
                                   const vector<Vehicle> &vehicles) {
  ASSERT_FALSE(isCollision(egoCar, vehicles))<< "COLLISION:" << endl << egoCar << vehicles[0];
  check();
}

void drive2Point(const Point &dst, EgoCar &egoCar, double dt,
                 const MapWaypoints &map_waypoints,
                 const vector<Vehicle> &vehicles,
                 const function<void(void)>& check) {

  check_and_assert_no_collision(check, egoCar, vehicles);

  const Point &src = egoCar.getPos_cart();
  egoCar.speed = distance(src, dst) / dt * 2.24;
  egoCar.setPos_cart(dst, map_waypoints);
  egoCar.yaw_deg = rad2deg(atan2(dst.y - src.y, dst.x - src.x));

  check_and_assert_no_collision(check, egoCar, vehicles);
}

void drive2Points(const vector<Point>& points, int numberOfUnprocessedElements,
                  double dt, const MapWaypoints& map_waypoints,
                  const function<void(void)>& check, EgoCar& egoCar,
                  const vector<Vehicle> &vehicles) {
  for (int i = 0; i < points.size() - numberOfUnprocessedElements; i++) {
    drive2Point(points[i], egoCar, dt, map_waypoints, vehicles, check);
  }
}

void updatePreviousData(const vector<Point>& points,
                        int numberOfUnprocessedElements, const Points& path,
                        const MapWaypoints& map_waypoints,
                        PreviousData& previousData, const EgoCar& egoCar) {
  previousData.previous_path_x.clear();
  previousData.previous_path_y.clear();
  for (int i = points.size() - numberOfUnprocessedElements; i < points.size();
      i++) {
    previousData.previous_path_x.push_back(path.xs[i]);
    previousData.previous_path_y.push_back(path.ys[i]);
  }
  previousData.end_path = getFrenet(
      points[points.size() - numberOfUnprocessedElements - 1],
      deg2rad(egoCar.yaw_deg), map_waypoints);
}

bool oneRoundDriven(const EgoCar &egoCar) {
  return egoCar.getPos_frenet().s > 6900;
}

void drive(ReferencePoint &refPoint, int &lane,
           const MapWaypoints &map_waypoints, EgoCar &egoCar,
           PreviousData &previousData, const vector<Vehicle> &vehicles,
           double dt, const function<void(void)> &check) {

  for (int i = 0; i < 1000 && !oneRoundDriven(egoCar); i++) {
    Points path = createPath(refPoint, lane, map_waypoints, egoCar,
                             previousData, vehicles, dt);
    vector<Point> points = test::getPoints(path, map_waypoints);
    int numberOfUnprocessedElements = 10;
    drive2Points(points, numberOfUnprocessedElements, dt, map_waypoints, check,
                 egoCar, vehicles);
    updatePreviousData(points, numberOfUnprocessedElements, path, map_waypoints,
                       previousData, egoCar);
  }
}

Point createCartVectorConnectingStartAndEnd(const Frenet &start,
                                            const Frenet &end,
                                            const MapWaypoints &map_waypoints) {
  Point start_cart = getXY(start, map_waypoints);
  Point end_cart = getXY(end, map_waypoints);
  return Point { end_cart.x - start_cart.x, end_cart.y - start_cart.y };
}

Vehicle createVehicle(int id, const Frenet &pos, const Frenet &v,
                      const MapWaypoints &map_waypoints) {
  Vehicle vehicle;
  vehicle.id = id;
  vehicle.pos_frenet = pos;
  vehicle.pos_cart = getXY(pos, map_waypoints);
  vehicle.vel = createCartVectorConnectingStartAndEnd(
      pos, Frenet { pos.s + v.s, pos.d + v.d }, map_waypoints);
  return vehicle;
}

}

TEST(PathPlanningTest, should_drive_in_same_lane) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel = 0;
  int lane = 1;
  Frenet pos = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(pos, map_waypoints);

  PreviousData previousData;
  vector<Vehicle> vehicles;

  double dt = 0.02;

// WHEN
  Points path = createPath(refPoint, lane, map_waypoints, egoCar, previousData,
                           vehicles, dt);

// THEN
  test::assert_car_drives_in_middle_of_lane(path, 1, map_waypoints);
  test::assert_car_drives_straight_ahead(path, map_waypoints);
}

TEST(PathPlanningTest, should_drive_with_max_50_mph) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel = 0;
  int lane = 1;
  Frenet pos = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(pos, map_waypoints);

  PreviousData previousData;
  vector<Vehicle> vehicles;

  double dt = 0.02;

// WHEN
  test::drive(refPoint, lane, map_waypoints, egoCar, previousData, vehicles, dt,
              [&egoCar]() {ASSERT_LT(egoCar.speed, 50);});

// THEN
}

TEST(PathPlanningTest, should_collide) {
  // GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  Frenet posCar = Frenet { 124.8336, getMiddleOfLane(1) };
  EgoCar egoCar = test::createEgoCar(posCar, map_waypoints);
  Vehicle vehicle = test::createVehicle(0,
                                        Frenet { posCar.s + test::carRadius / 2,
                                            posCar.d },
                                        Frenet { 0, 0 }, map_waypoints);

  // WHEN

  // THEN
  ASSERT_TRUE(test::isCollision(egoCar, vehicle));
}

TEST(PathPlanningTest, should_not_collide) {
  // GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel = 0;
  double dt = 0.02;
  PreviousData previousData;
  int lane = 1;
  Frenet posCar = Frenet { 124.8336 + 10 * (2 * test::carRadius),
      getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(posCar, map_waypoints);
  Vehicle vehicle = test::createVehicle(
      // TODO: Geschwindigkeitsvektor des Vehicles umsetzen, d.h. Vehicle fahren lassen.
      0, Frenet { posCar.s - 10 * (2 * test::carRadius), posCar.d }, Frenet { 5,
          0 },
      map_waypoints);
  vector<Vehicle> vehicles = { vehicle };

  // WHEN
  test::drive(refPoint, lane, map_waypoints, egoCar, previousData, vehicles, dt,
              [&egoCar, &vehicles]() {
                ASSERT_FALSE(test::isCollision(egoCar, vehicles));});

  // THEN
  // EgoCar: {pos_cart = {x = 909.48000000000002, y = 1128.6700000000001}, pos_frenet = {s = 124.8336, d = 6.1648329999999998}, yaw = 0, speed = 0}

}
