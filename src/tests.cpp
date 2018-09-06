#include "gtest/gtest.h"
#include "pathPlanner.h"

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

bool collision(const Points &egoPath, const vector<Vehicle> &vehicles) {
  return false;
}

EgoCar createEgoCar(const Frenet &pos, const MapWaypoints &map_waypoints) {
  EgoCar egoCar;
  egoCar.pos_frenet = pos;
  egoCar.pos_cart = getXY(pos.s, pos.d, map_waypoints);
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

void assert_car_drives_straight_ahead(const Points &path,
                                      const MapWaypoints &map_waypoints) {
  vector<double> distancesAlongRoad = map2<Frenet, double>(
      asFrenets(getPoints(path, map_waypoints), map_waypoints),
      [](const Frenet &frenet) {return frenet.s;});
  ASSERT_TRUE(
      std::is_sorted(distancesAlongRoad.begin(), distancesAlongRoad.end()));
}

}

TEST(PathPlanningTest, should_ego_drive_in_same_lane) {
// GIVEN
  MapWaypoints map_waypoints = read_map_waypoints();
  ReferencePoint refPoint;
  refPoint.vel = 0;
  int lane = 1;
  Frenet pos = Frenet { 124.8336, 2. + 4. * lane };
  EgoCar egoCar = test::createEgoCar(pos, map_waypoints);

  PreviousData previousData;
  vector<Vehicle> vehicles;

// WHEN
  Points path = createPath(refPoint, lane, map_waypoints, egoCar, previousData,
                           vehicles);

// THEN
  test::assert_car_drives_in_middle_of_lane(path, 1, map_waypoints);
  test::assert_car_drives_straight_ahead(path, map_waypoints);
}
