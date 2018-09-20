#ifndef TESTS_SIMULATOR_H_
#define TESTS_SIMULATOR_H_

#include <vector>

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

Vehicle createVehicle(int id, const Frenet& pos, const Frenet& vel_m_per_sec,
                      const CoordsConverter& coordsConverter) {
  Vehicle vehicle(coordsConverter);
  vehicle.id = id;
  vehicle.setPos_frenet(pos);
  vehicle.setVel_frenet_m_per_s(vel_m_per_sec);
  return vehicle;
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

class Simulator {
 public:
  // TODO: make check a parameter of drive()
  Simulator(ReferencePoint& refPoint, Lane& lane,
            const CoordsConverter& coordsConverter, EgoCar& egoCar,
            PreviousData& previousData, vector<Vehicle>& vehicles, double dt,
            int minSecs2Drive);
  void drive(function<void(void)> check);

 private:
  double driveEgoCarAndVehicles(function<void(void)> check);
  double drive2PointsOfEgoCarAndDriveVehicles(
      const vector<Point>& points, int numberOfUnprocessedPathElements,
      function<void(void)> check);
  void driveVehicles();
  void driveVehicle(Vehicle& vehicle);
  void drive2PointOfEgoCar(const Point& dst, function<void(void)> check);
  void updatePreviousData(const vector<Point>& points,
                          int numberOfUnprocessedPathElements,
                          const Path& path);
  bool oneRoundDriven();

  ReferencePoint& refPoint;
  Lane& lane;
  const CoordsConverter& coordsConverter;
  EgoCar& egoCar;
  PreviousData& previousData;
  vector<Vehicle>& vehicles;
  double dt;
  int minSecs2Drive;
  function<void(void)> check;
};

Simulator::Simulator(ReferencePoint& _refPoint, Lane& _lane,
                     const CoordsConverter& _coordsConverter, EgoCar& _egoCar,
                     PreviousData& _previousData, vector<Vehicle>& _vehicles,
                     double _dt, int _minSecs2Drive)
    : refPoint(_refPoint),
      lane(_lane),
      coordsConverter(_coordsConverter),
      egoCar(_egoCar),
      previousData(_previousData),
      vehicles(_vehicles),
      dt(_dt),
      minSecs2Drive(_minSecs2Drive) {
}

void Simulator::drive(function<void(void)> check) {
  double secsDriven = 0;
  while ((secsDriven <= minSecs2Drive || minSecs2Drive == NO_VALUE)
      && !oneRoundDriven()) {
    secsDriven += driveEgoCarAndVehicles(check);
  }
}

bool Simulator::oneRoundDriven() {
  return egoCar.getPos_frenet().s > 6900;
}

double Simulator::driveEgoCarAndVehicles(function<void(void)> check) {
  PathPlanner pathPlanner(coordsConverter, refPoint, lane, dt);
  Path path = pathPlanner.createPath(egoCar, previousData, vehicles);
  int numberOfUnprocessedPathElements = 10;
  double secsDriven = drive2PointsOfEgoCarAndDriveVehicles(
      path.points, numberOfUnprocessedPathElements, check);
  updatePreviousData(path.points, numberOfUnprocessedPathElements, path);
  return secsDriven;
}

void Simulator::updatePreviousData(const vector<Point>& points,
                                   int numberOfUnprocessedPathElements,
                                   const Path& path) {
  previousData.previous_path.points.clear();
  for (int i = points.size() - numberOfUnprocessedPathElements;
      i < points.size(); i++) {
    previousData.previous_path.points.push_back(path.points[i]);
  }
  previousData.end_path = coordsConverter.getFrenet(
      points[points.size() - numberOfUnprocessedPathElements - 1]);
}

double Simulator::drive2PointsOfEgoCarAndDriveVehicles(
    const vector<Point>& points, int numberOfUnprocessedPathElements,
    function<void(void)> check) {

  int numberOfProcessedPathElements = points.size()
      - numberOfUnprocessedPathElements;
  for (int i = 0; i < numberOfProcessedPathElements; i++) {
    driveVehicles();
    drive2PointOfEgoCar(points[i], check);
  }

  double secsDriven = numberOfProcessedPathElements * dt;
  return secsDriven;
}

void Simulator::driveVehicles() {
  for (Vehicle& vehicle : vehicles) {
    driveVehicle(vehicle);
  }
}

void Simulator::driveVehicle(Vehicle& vehicle) {
  const Frenet vel_frenet = vehicle.getVel_frenet_m_per_s();
  vehicle.setPos_frenet(vehicle.getPos_frenet() + (vel_frenet * dt));
  // GTEST_COUT<< "vehicle: " << vehicle.getPos_frenet() << endl;
}

void Simulator::drive2PointOfEgoCar(const Point& dst,
                                    function<void(void)> check) {
  const Point& src = egoCar.getPos_cart();
  egoCar.speed_mph = meter_per_sec2mph(src.distanceTo(dst) / dt);
  egoCar.setPos_cart(dst);
  egoCar.yaw_deg = rad2deg((dst - src).getHeading());
  // GTEST_COUT<< "egoCar: " << egoCar.getPos_frenet() << endl;

  ASSERT_FALSE(isCollision(egoCar, vehicles)) << "COLLISION:" << endl << egoCar
      << vehicles[0];
  check();
}

}

#endif /* TESTS_SIMULATOR_H_ */
