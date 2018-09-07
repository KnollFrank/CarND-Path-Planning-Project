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

bool isCollision(const Point &carPos, const Vehicle &vehicle) {
  const double carRadius = 5;
  return distance(carPos, vehicle.pos_cart) <= 2 * carRadius;
}

bool isCollision(const Point &carPos, const vector<Vehicle> &vehicles) {
  return std::any_of(
      vehicles.cbegin(), vehicles.cend(),
      [&carPos](const Vehicle &vehicle) {return isCollision(carPos, vehicle);});
}

EgoCar createEgoCar(const Frenet &pos, const MapWaypoints &map_waypoints) {
  EgoCar egoCar;
  egoCar.pos_frenet = pos;
  egoCar.pos_cart = getXY(pos, map_waypoints);
  egoCar.yaw = 0;
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

void drive2Point(const Point &dst, EgoCar &egoCar, double dt,
                 const MapWaypoints &map_waypoints,
                 const vector<Vehicle> &vehicles,
                 const function<void(void)>& check) {
  ASSERT_FALSE(isCollision(egoCar.pos_cart, vehicles))<< "COLLISION";
  check();

  Point &src = egoCar.pos_cart;
  egoCar.speed = distance(src, dst) / dt * 2.24;
  egoCar.pos_cart = dst;
  egoCar.pos_frenet = getFrenet(dst, 0, map_waypoints);
  // egoCar.yaw = ?;

  ASSERT_FALSE(isCollision(egoCar.pos_cart, vehicles)) << "COLLISION";
  check();
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
                        PreviousData& previousData) {
  previousData.previous_path_x.clear();
  previousData.previous_path_y.clear();
  for (int i = points.size() - numberOfUnprocessedElements; i < points.size();
      i++) {
    previousData.previous_path_x.push_back(path.xs[i]);
    previousData.previous_path_y.push_back(path.ys[i]);
  }
  previousData.end_path = getFrenet(
      points[points.size() - numberOfUnprocessedElements - 1], 0,
      map_waypoints);
}

void drive(ReferencePoint &refPoint, int &lane,
           const MapWaypoints &map_waypoints, EgoCar &egoCar,
           PreviousData &previousData, const vector<Vehicle> &vehicles,
           double dt, const function<void(void)> &check) {

  for (int i = 0; i < 1000; i++) {
    Points path = createPath(refPoint, lane, map_waypoints, egoCar,
                             previousData, vehicles, dt);
    vector<Point> points = test::getPoints(path, map_waypoints);
    int numberOfUnprocessedElements = 10;
    drive2Points(points, numberOfUnprocessedElements, dt, map_waypoints, check,
                 egoCar, vehicles);
    updatePreviousData(points, numberOfUnprocessedElements, path, map_waypoints,
                       previousData);
  }
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
              [&egoCar]() { {ASSERT_LT(egoCar.speed, 50);}});

// THEN
}

TEST(PathPlanningTest, should_node_collide) {

}

TEST(PathPlanningTest, should_collide) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel = 0;
  int lane = 1;
  Frenet pos = Frenet { 124.8336, getMiddleOfLane(lane) };
  EgoCar egoCar = test::createEgoCar(pos, map_waypoints);

  PreviousData previousData;

  vector<Vehicle> vehicles;
  Vehicle vehicle;
  vehicle.id = 0;
  vehicle.pos_frenet = pos;
  vehicle.pos_cart = getXY(pos, map_waypoints);
  vehicle.vx = 0;
  vehicle.vy = 0;
  vehicles.push_back(vehicle);

  double dt = 0.02;

// WHEN
  ASSERT_TRUE(test::isCollision(egoCar.pos_cart, vehicles[0]));

//  test::drive(
//      refPoint,
//      lane,
//      map_waypoints,
//      egoCar,
//      previousData,
//      vehicles,
//      dt,
//      [&egoCar, &vehicles]() { {ASSERT_TRUE(test::isCollision(egoCar.pos_cart, vehicles));}});

// THEN
}
