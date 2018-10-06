#ifndef TESTS_VEHICLEDRIVER_H_
#define TESTS_VEHICLEDRIVER_H_

#include "../car.h"
#include "../coords/frenet.h"
#include "../coords/frenetCart.h"

class VehicleDriver {

 public:
  VehicleDriver(const CoordsConverter& coordsConverter, double dt);
  virtual ~VehicleDriver() {};
  virtual void driveVehicle(Vehicle& vehicle) = 0;

 protected:
  const CoordsConverter& coordsConverter;
  double dt;
};

VehicleDriver::VehicleDriver(const CoordsConverter& _coordsConverter,
                             double _dt)
    : coordsConverter(_coordsConverter),
      dt(_dt) {
}

class StandardVehicleDriver : public VehicleDriver {

 public:
  StandardVehicleDriver(const CoordsConverter& coordsConverter, double dt);
  void driveVehicle(Vehicle& vehicle);
};

StandardVehicleDriver::StandardVehicleDriver(
    const CoordsConverter& coordsConverter, double dt)
    : VehicleDriver(coordsConverter, dt) {
}

void StandardVehicleDriver::driveVehicle(Vehicle& vehicle) {
  vehicle.setPos(
      FrenetCart(
          vehicle.getPos().getFrenet() + (vehicle.getVel_frenet_m_per_s() * dt),
          coordsConverter));
// GTEST_COUT<< "vehicle: " << vehicle.getPos_frenet() << endl;
}

#endif /* TESTS_VEHICLEDRIVER_H_ */
