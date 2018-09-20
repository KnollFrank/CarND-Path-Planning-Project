#ifndef TESTS_SIMULATOR_H_
#define TESTS_SIMULATOR_H_

#include <vector>

constexpr int NO_VALUE = -1;

#define GTEST_COUT std::cerr

class Simulator {
 public:
  Simulator(ReferencePoint& refPoint, Lane& lane,
            const CoordsConverter& coordsConverter, EgoCar& egoCar,
            PreviousData& previousData, vector<Vehicle>& vehicles, double dt,
            int minSecs2Drive);
  void drive(function<void(void)> afterEachMovementOfEgoCar);
  static bool isCollision(const EgoCar& egoCar, const Vehicle& vehicle);
  static bool isCollision(const EgoCar& egoCar,
                          const vector<Vehicle>& vehicles);

 private:
  double driveEgoCarAndVehicles(function<void(void)> afterEachMovementOfEgoCar);
  double drive2PointsOfEgoCarAndDriveVehicles(
      const vector<Point>& points, int numberOfUnprocessedPathElements,
      function<void(void)> afterEachMovementOfEgoCar);
  void driveVehicles();
  void driveVehicle(Vehicle& vehicle);
  void drive2PointOfEgoCar(const Point& dst,
                           function<void(void)> afterEachMovementOfEgoCar);
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
  function<void(void)> afterEachMovementOfEgoCar;
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

void Simulator::drive(function<void(void)> afterEachMovementOfEgoCar) {
  double secsDriven = 0;
  while ((secsDriven <= minSecs2Drive || minSecs2Drive == NO_VALUE)
      && !oneRoundDriven()) {
    secsDriven += driveEgoCarAndVehicles(afterEachMovementOfEgoCar);
  }
}

bool Simulator::oneRoundDriven() {
  return egoCar.getPos_frenet().s > 6900;
}

double Simulator::driveEgoCarAndVehicles(
    function<void(void)> afterEachMovementOfEgoCar) {
  PathPlanner pathPlanner(coordsConverter, refPoint, lane, dt);
  Path path = pathPlanner.createPath(egoCar, previousData, vehicles);
  int numberOfUnprocessedPathElements = 10;
  double secsDriven = drive2PointsOfEgoCarAndDriveVehicles(
      path.points, numberOfUnprocessedPathElements, afterEachMovementOfEgoCar);
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
    function<void(void)> afterEachMovementOfEgoCar) {

  int numberOfProcessedPathElements = points.size()
      - numberOfUnprocessedPathElements;
  for (int i = 0; i < numberOfProcessedPathElements; i++) {
    driveVehicles();
    drive2PointOfEgoCar(points[i], afterEachMovementOfEgoCar);
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

void Simulator::drive2PointOfEgoCar(
    const Point& dst, function<void(void)> afterEachMovementOfEgoCar) {
  const Point& src = egoCar.getPos_cart();
  egoCar.speed_mph = meter_per_sec2mph(src.distanceTo(dst) / dt);
  egoCar.setPos_cart(dst);
  egoCar.yaw_deg = rad2deg((dst - src).getHeading());
  // GTEST_COUT<< "egoCar: " << egoCar.getPos_frenet() << endl;

  ASSERT_FALSE(isCollision(egoCar, vehicles)) << "COLLISION:" << endl << egoCar
      << vehicles[0];
  afterEachMovementOfEgoCar();
}

bool Simulator::isCollision(const EgoCar& egoCar, const Vehicle& vehicle) {
  return egoCar.getPos_cart().distanceTo(vehicle.getPos_cart()) <= EgoCar::carSize();
}

bool Simulator::isCollision(const EgoCar& egoCar,
                            const vector<Vehicle>& vehicles) {
  return std::any_of(
      vehicles.cbegin(), vehicles.cend(),
      [&](const Vehicle& vehicle) {return isCollision(egoCar, vehicle);});
}

#endif /* TESTS_SIMULATOR_H_ */
