#ifndef TESTS_PATHPLANNERTEST_H_
#define TESTS_PATHPLANNERTEST_H_

#include "../pathPlanner.h"

#include <gtest/gtest.h>
#include <gtest/gtest-message.h>
#include <gtest/internal/gtest-internal.h>
#include <algorithm>
#include <experimental/optional>
#include <functional>
#include <vector>

#include "../egoCar.h"
#include "../coords/coordsConverter.h"
#include "../coords/frenet.h"
#include "../coords/frenetCart.h"
#include "../coords/waypoints.h"
#include "../funs.h"
#include "../lane.h"
#include "../path.h"
#include "../previousData.h"
#include "simulator.h"
#include "vehicleDriver.h"

using namespace std;
using namespace std::experimental;

#define GTEST_COUT std::cerr

class PathPlannerTest : public ::testing::Test {

 protected:
  static constexpr double START_S_COORD = 124.8336;

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
    return createSimulator(lane, egoCar, vehicles, minSecs2Drive,
                           new StandardVehicleDriver(*coordsConverter));
  }

  Simulator createSimulator(Lane& lane, EgoCar& egoCar,
                            vector<Vehicle>& vehicles,
                            std::experimental::optional<int> minSecs2Drive,
                            VehicleDriver* vehicleDriver) {
    const double dt = 0.02;
    const double speed_limit_mph = 50;
    return Simulator(refPoint, lane, *coordsConverter, egoCar, previousData,
                     vehicles, dt, minSecs2Drive, vehicleDriver,
                     speed_limit_mph);
  }

  EgoCar createEgoCar(const Frenet& pos) {
    EgoCar egoCar(*coordsConverter);
    egoCar.setPos(FrenetCart(pos, *coordsConverter));
    egoCar.yaw_deg = 0;
    egoCar.speed_mph = 0;
    return egoCar;
  }

  Vehicle createVehicle(int id, const Frenet& pos, double vel_m_per_sec) {
    Vehicle vehicle(id, FrenetCart(pos, *coordsConverter), *coordsConverter);
    vehicle.setVel_frenet_m_per_s(Frenet { vel_m_per_sec, 0 });
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

  void should_overtake_two_parallel_vehicles(const Lane& anotherVehiclesLane);

  void should_overtake_vehicle_by_switching_to_more_free_lane(
      double leftVehiclesSOffset2EgoCar, double rightVehiclesSOffset2EgoCar,
      const Lane& expectedlaneOfEgoCarWhileOvertakingVehicle);

  Frenet parallelToVehicleInLane(const Vehicle& vehicle, const Lane& lane) {
    return Frenet { vehicle.getPos().getFrenet().s, getMiddleOfLane(lane) };
  }

  Frenet egoCarPlusMeters(const EgoCar& egoCar, const double s) {
    return egoCar.getPos().getFrenet() + Frenet { s, 0 };
  }

  MapWaypoints mapWaypoints;
  CoordsConverter* coordsConverter;
  ReferencePoint refPoint;
  PreviousData previousData;
};

TEST_F(PathPlannerTest, should_drive_in_same_lane) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });
  vector<Vehicle> vehicles;

  PathPlanner pathPlanner(*coordsConverter, refPoint, lane, 0.02, 50, vehicles, egoCar);

// WHEN
  Path path = pathPlanner.createPath(previousData);

// THEN
  assert_car_drives_in_middle_of_lane(path, Lane::MIDDLE);
  assert_car_drives_straight_ahead(path);
}

TEST_F(PathPlannerTest, should_drive_in_same_lane_without_incidents) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });
  vector<Vehicle> vehicles;

  Simulator simulator = createSimulator(lane, egoCar, vehicles,
                                        std::experimental::nullopt);

// WHEN
  simulator.run([&]() {
    ASSERT_NEAR(2 + 4 * Lane::MIDDLE, egoCar.getPos().getFrenet().d, 0.001);
  });

// THEN
}

TEST_F(PathPlannerTest, should_keep_speed_close_to_speed_limit) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });
  vector<Vehicle> vehicles;

  Simulator simulator = createSimulator(lane, egoCar, vehicles,
                                        std::experimental::nullopt);

// WHEN
  vector<double> velocities;
  simulator.run([&]() {
    velocities.push_back(egoCar.speed_mph);
  });

// THEN
//  for (double velocity : velocities) {
//    GTEST_COUT<< velocity << endl;
//  }
  double real_speed_limit_mph = 0.89 * simulator.speed_limit_mph;
  auto speedLimitReached = find_if(
      begin(velocities), end(velocities), [&](const double& velocity) {
        return areNear(velocity, real_speed_limit_mph, 1);
      });
  ASSERT_NE(speedLimitReached, end(velocities))<< "should reach speed limit";
  ASSERT_TRUE(all_of(
          speedLimitReached,
          end(velocities),
          [&](const double& velocity) {
            return areNear(velocity, real_speed_limit_mph, 3);
          }))<< "should keep speed close to speed limit";
}

TEST_F(PathPlannerTest, should_collide) {
// GIVEN
  EgoCar egoCar = createEgoCar(
      Frenet { START_S_COORD, getMiddleOfLane(Lane::MIDDLE) });
  Vehicle vehicle = createVehicle(
      0, egoCarPlusMeters(egoCar, egoCar.getShape().getHeight() / 2), 0);

// WHEN

// THEN
  ASSERT_TRUE(Simulator::isCollision(egoCar, vehicle));
}

TEST_F(PathPlannerTest, should_not_collide) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });
  Vehicle vehicle = createVehicle(
      0, egoCarPlusMeters(egoCar, 10 * egoCar.getShape().getHeight()), 5);
  vector<Vehicle> vehicles = { vehicle };

  Simulator simulator = createSimulator(lane, egoCar, vehicles,
                                        std::experimental::nullopt);

// WHEN
  simulator.run([&]() {
    ASSERT_FALSE(Simulator::isCollision(egoCar, vehicles));});

// THEN
}

TEST_F(PathPlannerTest, should_drive_behind_three_parallel_vehicles) {
  // GIVEN
  Lane lane = Lane::MIDDLE;

  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });

  Vehicle middle = createVehicle(0, egoCarPlusMeters(egoCar, 35),
                                 mph2meter_per_sec(35));

  Vehicle left = createVehicle(1, parallelToVehicleInLane(middle, Lane::LEFT),
                               middle.getVel_frenet_m_per_s().s);

  Vehicle right = createVehicle(2, parallelToVehicleInLane(middle, Lane::RIGHT),
                                middle.getVel_frenet_m_per_s().s);

  vector<Vehicle> vehicles { middle, left, right };

  Simulator simulator = createSimulator(lane, egoCar, vehicles,
                                        std::experimental::nullopt);

// WHEN
  simulator.run([&]() {
    ASSERT_FALSE(Simulator::isCollision(egoCar, vehicles));
  });

// THEN
}

TEST_F(PathPlannerTest, should_overtake_vehicle) {
// GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });
  Vehicle vehicle = createVehicle(0, egoCarPlusMeters(egoCar, 35),
                                  mph2meter_per_sec(5));
  vector<Vehicle> vehicles = { vehicle };

  Simulator simulator = createSimulator(lane, egoCar, vehicles, 60);

// WHEN
  bool egoCarOvertakesVehicle = false;
  simulator.run(
      [&]() {
        bool overtaken = egoCar.getPos().getFrenet().s > vehicles[0].getPos().getFrenet().s;
        egoCarOvertakesVehicle = egoCarOvertakesVehicle || overtaken;
      });

// THEN
  ASSERT_TRUE(egoCarOvertakesVehicle)<< "egoCar should overtake vehicle";
}

void PathPlannerTest::should_overtake_two_parallel_vehicles(
    const Lane& anotherVehiclesLane) {
  // GIVEN
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });
  Vehicle vehicle2Overtake = createVehicle(0, egoCarPlusMeters(egoCar, 35),
                                           mph2meter_per_sec(5));

  Vehicle vehicleInAnotherLane = createVehicle(
      1, parallelToVehicleInLane(vehicle2Overtake, anotherVehiclesLane),
      vehicle2Overtake.getVel_frenet_m_per_s().s);
  vector<Vehicle> vehicles { vehicle2Overtake, vehicleInAnotherLane };

  Simulator simulator = createSimulator(lane, egoCar, vehicles, 60);

  // WHEN
  bool egoCarOvertakesVehicle = false;
  simulator.run(
      [&]() {
        bool overtaken = egoCar.getPos().getFrenet().s > vehicles[0].getPos().getFrenet().s;
        egoCarOvertakesVehicle = egoCarOvertakesVehicle || overtaken;
      });

  // THEN
  ASSERT_TRUE(egoCarOvertakesVehicle)<< "egoCar should overtake vehicle";
}

TEST_F(PathPlannerTest,
    should_overtake_two_parallel_vehicles_middleLane_leftLane) {
  should_overtake_two_parallel_vehicles(Lane::LEFT);
}

TEST_F(PathPlannerTest,
    should_overtake_two_parallel_vehicles_middleLane_rightLane) {
  should_overtake_two_parallel_vehicles(Lane::RIGHT);
}

class VehicleDriverDrivingOneVehicleBehindEgoCar : public VehicleDriver {

 public:
  VehicleDriverDrivingOneVehicleBehindEgoCar(
      const CoordsConverter& coordsConverter,
      int _idOfVehicle2DriveBehindEgoCar)
      : VehicleDriver(coordsConverter),
        idOfVehicle2DriveBehindEgoCar(_idOfVehicle2DriveBehindEgoCar) {
    delegate = new StandardVehicleDriver(coordsConverter);
  }

  virtual ~VehicleDriverDrivingOneVehicleBehindEgoCar() {
    delete delegate;
  }

  FrenetCart getNewPos(const Vehicle& vehicle, const EgoCar& egoCar, double dt)
      override {
    return
        vehicle.id == idOfVehicle2DriveBehindEgoCar ?
            getPosBehindEgoCar(vehicle, egoCar) :
            delegate->getNewPos(vehicle, egoCar, dt);
  }

  FrenetCart getPosBehindEgoCar(const Vehicle& vehicle, const EgoCar& egoCar) {
    Frenet posBehindEgoCar = Frenet { egoCar.getPos().getFrenet().s - 1, vehicle
        .getPos().getFrenet().d };
    return FrenetCart(posBehindEgoCar, coordsConverter);
  }

 private:
  VehicleDriver *delegate;
  int idOfVehicle2DriveBehindEgoCar;
};

TEST_F(PathPlannerTest, should_not_overtake_two_parallel_vehicles_when_third_car_comes_from_behind_in_free_lane) {
  Lane lane = Lane::MIDDLE;
  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });

  Vehicle middle = createVehicle(0, egoCarPlusMeters(egoCar, 35),
                                 mph2meter_per_sec(35));

  Vehicle left = createVehicle(1, parallelToVehicleInLane(middle, Lane::LEFT),
                               middle.getVel_frenet_m_per_s().s);

  Vehicle right = createVehicle(2, parallelToVehicleInLane(middle, Lane::RIGHT),
                                middle.getVel_frenet_m_per_s().s);

  vector<Vehicle> vehicles { middle, left, right };

  Simulator simulator = createSimulator(
      lane,
      egoCar,
      vehicles,
      std::experimental::nullopt,
      new VehicleDriverDrivingOneVehicleBehindEgoCar(*coordsConverter,
                                                     right.id));

  // WHEN & THEN
  simulator.run([&]() {
    ASSERT_FALSE(Simulator::isCollision(egoCar, vehicles));
  });
}

void PathPlannerTest::should_overtake_vehicle_by_switching_to_more_free_lane(
    double leftVehiclesSOffset2EgoCar, double rightVehiclesSOffset2EgoCar,
    const Lane& expectedlaneOfEgoCarWhileOvertakingVehicle) {

  // GIVEN
  Lane lane = Lane::MIDDLE;

  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });

  Vehicle middle = createVehicle(0, egoCarPlusMeters(egoCar, 35),
                                 mph2meter_per_sec(35));
  Vehicle left = createVehicle(
      1, parallelToVehicleInLane(middle, Lane::LEFT) + Frenet {
          leftVehiclesSOffset2EgoCar, 0 },
      middle.getVel_frenet_m_per_s().s);

  Vehicle right = createVehicle(
      2, parallelToVehicleInLane(middle, Lane::RIGHT) + Frenet {
          rightVehiclesSOffset2EgoCar, 0 },
      middle.getVel_frenet_m_per_s().s);

  vector<Vehicle> vehicles { middle, left, right };

  Simulator simulator = createSimulator(lane, egoCar, vehicles,
                                        std::experimental::nullopt);

  // WHEN
  bool egoCarOvertakesVehicle = false;
  std::experimental::optional<Lane> laneOfEgoCarWhileOvertakingVehicle =
      std::experimental::nullopt;
  simulator.run(
      [&]() {
        bool overtaken = egoCar.getPos().getFrenet().s > vehicles[0].getPos().getFrenet().s;
        if(overtaken && !laneOfEgoCarWhileOvertakingVehicle) {
          laneOfEgoCarWhileOvertakingVehicle = getLane(egoCar.getPos().getFrenet().d);
        }
        egoCarOvertakesVehicle = egoCarOvertakesVehicle || overtaken;
      });

  // THEN
  ASSERT_TRUE(egoCarOvertakesVehicle)<< "egoCar should overtake vehicle";
  ASSERT_EQ(expectedlaneOfEgoCarWhileOvertakingVehicle,
            *laneOfEgoCarWhileOvertakingVehicle);
}

TEST_F(PathPlannerTest,
    should_overtake_vehicle_by_switching_to_more_free_right_lane) {

  should_overtake_vehicle_by_switching_to_more_free_lane(100, 200, Lane::RIGHT);
}

TEST_F(PathPlannerTest,
    should_overtake_vehicle_by_switching_to_more_free_left_lane) {

  should_overtake_vehicle_by_switching_to_more_free_lane(200, 100, Lane::LEFT);
}

TEST_F(PathPlannerTest,
    should_overtake_vehicle_by_switching_to_more_free_right_lane2) {

  // GIVEN
  Lane lane = Lane::MIDDLE;

  EgoCar egoCar = createEgoCar(Frenet { START_S_COORD, getMiddleOfLane(lane) });

  Vehicle middle = createVehicle(0, egoCarPlusMeters(egoCar, 35),
                                 mph2meter_per_sec(35));
  Vehicle left = createVehicle(
      1, parallelToVehicleInLane(middle, Lane::LEFT) + Frenet { 100, 0 },
      middle.getVel_frenet_m_per_s().s);

  Vehicle right = createVehicle(
      2, parallelToVehicleInLane(middle, Lane::RIGHT) + Frenet { -100, 0 }, 0);

  vector<Vehicle> vehicles { middle, left, right };

  Simulator simulator = createSimulator(lane, egoCar, vehicles,
                                        std::experimental::nullopt);

  // WHEN
  bool egoCarOvertakesVehicle = false;
  std::experimental::optional<Lane> laneOfEgoCarWhileOvertakingVehicle =
      std::experimental::nullopt;
  simulator.run(
      [&]() {
        bool overtaken = egoCar.getPos().getFrenet().s > vehicles[0].getPos().getFrenet().s;
        if(overtaken && !laneOfEgoCarWhileOvertakingVehicle) {
          laneOfEgoCarWhileOvertakingVehicle = getLane(egoCar.getPos().getFrenet().d);
        }
        egoCarOvertakesVehicle = egoCarOvertakesVehicle || overtaken;
      });

  // THEN
  ASSERT_TRUE(egoCarOvertakesVehicle)<< "egoCar should overtake vehicle";
  ASSERT_EQ(Lane::RIGHT, *laneOfEgoCarWhileOvertakingVehicle);
}

#endif
