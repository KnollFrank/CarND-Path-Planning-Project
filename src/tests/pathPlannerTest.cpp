#include "../pathPlanner.h"

#include <ext/type_traits.h>
#include <gtest/gtest.h>
#include <gtest/gtest-message.h>
#include <gtest/internal/gtest-internal.h>
#include <algorithm>
#include <iterator>
#include <vector>

#include "../car.h"
#include "../coords/coordsConverter.h"
#include "../coords/frenet.h"
#include "../coords/frenetCart.h"
#include "../coords/waypoints.h"
#include "../funs.h"
#include "../lane.h"
#include "../path.h"
#include "../previousData.h"
#include "simulator.h"

using namespace std;
using namespace std::experimental;

#define GTEST_COUT std::cerr

class PathPlannerTest : public ::testing::Test {

 protected:
  void SetUp() override {
    mapWaypoints = MapWaypoints::load();
    coordsConverter = new CoordsConverter(mapWaypoints);
    refPoint.vel_mph = 0;
  }

  void TearDown() override {
    delete coordsConverter;
  }

  Simulator createSimulator(Lane& lane, EgoCar& egoCar,
                            vector<Vehicle>& vehicles,
                            std::experimental::optional<int> minSecs2Drive) {
    return Simulator(refPoint, lane, *coordsConverter, egoCar, previousData,
                     vehicles, 0.02, minSecs2Drive);
  }

  EgoCar createEgoCar(const Frenet& pos, double dt) {
    EgoCar egoCar(*coordsConverter, dt);
    egoCar.setPos(FrenetCart(pos, *coordsConverter));
    egoCar.yaw_deg = 0;
    egoCar.speed_mph = 0;
    return egoCar;
  }

  Vehicle createVehicle(int id, const Frenet& pos,
                        const Frenet& vel_m_per_sec) {
    Vehicle vehicle(id, FrenetCart(pos, *coordsConverter), *coordsConverter);
    vehicle.setVel_frenet_m_per_s(vel_m_per_sec);
    return vehicle;
  }

  void assert_car_drives_in_middle_of_lane(const Path& path, Lane lane) {
    for (const FrenetCart& frenet : path.points) {
      ASSERT_NEAR(2 + 4 * lane, frenet.getFrenet().d, 0.001);
    }
  }

  vector<double> getDistancesAlongRoad(const Path& path) {

    return map2<FrenetCart, double>(
        path.points,
        [&](const FrenetCart& frenet) {return frenet.getFrenet().s;});
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
  // TODO: share 0.02 within EgoCar with Pathplanner
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) },
                               0.02);
  vector<Vehicle> vehicles;

  PathPlanner pathPlanner(*coordsConverter, refPoint, lane, 0.02);

// WHEN
  Path path = pathPlanner.createPath(egoCar, previousData, vehicles);

// THEN
  assert_car_drives_in_middle_of_lane(path, Lane::MIDDLE);
  assert_car_drives_straight_ahead(path);
}

TEST_F(PathPlannerTest, should_drive_in_same_lane_without_incidents) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) },
                               0.02);
  vector<Vehicle> vehicles;

  Simulator simulator = createSimulator(lane, egoCar, vehicles,
                                        std::experimental::nullopt);

// WHEN
  simulator.drive([&]() {
    ASSERT_NEAR(2 + 4 * Lane::MIDDLE, egoCar.getPos().getFrenet().d, 0.001);});

// THEN
}

TEST_F(PathPlannerTest, should_collide) {
// GIVEN
  EgoCar egoCar = createEgoCar(
      Frenet { 124.8336, getMiddleOfLane(Lane::MIDDLE) }, 0.02);
  Vehicle vehicle = createVehicle(
      0, egoCar.getPos().getFrenet() + Frenet { EgoCar::carRadius() / 2, 0 },
      Frenet::zero());

// WHEN

// THEN
  ASSERT_TRUE(Simulator::isCollision(egoCar, vehicle));
}

TEST_F(PathPlannerTest, should_not_collide) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) },
                               0.02);
  Vehicle vehicle = createVehicle(
      0, egoCar.getPos().getFrenet() + Frenet { 10 * EgoCar::carSize(), 0 },
      Frenet { 5, 0 });
  vector<Vehicle> vehicles = { vehicle };

  Simulator simulator = createSimulator(lane, egoCar, vehicles,
                                        std::experimental::nullopt);

// WHEN
  simulator.drive([&]() {
    ASSERT_FALSE(Simulator::isCollision(egoCar, vehicles));});

// THEN
}

TEST_F(PathPlannerTest, should_drive_behind_three_parallel_vehicles) {
  // GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) },
                               0.02);
  // TODO: refactor
  Vehicle vehicleInMiddleLane = createVehicle(
      0, egoCar.getPos().getFrenet() + Frenet { 35, 0 }, Frenet {
          mph2meter_per_sec(5), 0 });
  Vehicle vehicleInLeftLane = createVehicle(
      1,
      Frenet { vehicleInMiddleLane.getPos().getFrenet().s, getMiddleOfLane(
          Lane::LEFT) },
      vehicleInMiddleLane.getVel_frenet_m_per_s());
  Vehicle vehicleInRightLane = createVehicle(
      2,
      Frenet { vehicleInMiddleLane.getPos().getFrenet().s, getMiddleOfLane(
          Lane::RIGHT) },
      vehicleInMiddleLane.getVel_frenet_m_per_s());
  vector<Vehicle> vehicles { vehicleInMiddleLane, vehicleInLeftLane, vehicleInRightLane };


  Simulator simulator = createSimulator(lane, egoCar, vehicles,
                                        std::experimental::nullopt);

// WHEN
  simulator.drive([&]() {
    ASSERT_FALSE(Simulator::isCollision(egoCar, vehicles));});

// THEN
}

TEST_F(PathPlannerTest, should_overtake_vehicle) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) },
                               0.02);
  Vehicle vehicle = createVehicle(0, egoCar.getPos().getFrenet() + Frenet { 35,
      0 },
                                  Frenet { mph2meter_per_sec(5), 0 });
  vector<Vehicle> vehicles = { vehicle };

  Simulator simulator = createSimulator(lane, egoCar, vehicles, 60);

// WHEN
  bool egoCarOvertakesVehicle = false;
  simulator.drive(
      [&]() {
        bool overtaken = egoCar.getPos().getFrenet().s > vehicles[0].getPos().getFrenet().s;
        egoCarOvertakesVehicle = egoCarOvertakesVehicle || overtaken;
      });

// THEN
  ASSERT_TRUE(egoCarOvertakesVehicle)<< "egoCar should overtake vehicle";
}

TEST_F(PathPlannerTest, should_overtake_two_parallel_vehicles) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { 124.8336, getMiddleOfLane(lane) },
                               0.02);
  Vehicle vehicle2Overtake = createVehicle(
      0, egoCar.getPos().getFrenet() + Frenet { 35, 0 }, Frenet {
          mph2meter_per_sec(5), 0 });
  Vehicle vehicleInLeftLane = createVehicle(
      1,
      Frenet { vehicle2Overtake.getPos().getFrenet().s, getMiddleOfLane(
          Lane::LEFT) },
      vehicle2Overtake.getVel_frenet_m_per_s());
  vector<Vehicle> vehicles { vehicle2Overtake, vehicleInLeftLane };

  Simulator simulator = createSimulator(lane, egoCar, vehicles, 60);

// WHEN
  bool egoCarOvertakesVehicle = false;
  simulator.drive(
      [&]() {
        bool overtaken = egoCar.getPos().getFrenet().s > vehicles[0].getPos().getFrenet().s;
        egoCarOvertakesVehicle = egoCarOvertakesVehicle || overtaken;
      });

  // THEN
  ASSERT_TRUE(egoCarOvertakesVehicle)<< "egoCar should overtake vehicle";
}
