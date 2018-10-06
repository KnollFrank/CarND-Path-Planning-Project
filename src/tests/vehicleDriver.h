#ifndef TESTS_VEHICLEDRIVER_H_
#define TESTS_VEHICLEDRIVER_H_

#include "../car.h"
#include "../coords/frenet.h"
#include "../coords/frenetCart.h"

class VehicleDriver {

 public:
  VehicleDriver(const CoordsConverter& coordsConverter);
  virtual ~VehicleDriver() {
  }

  virtual void driveVehicle(Vehicle& vehicle, const EgoCar& egoCar,
                            double dt) = 0;

 protected:
  const CoordsConverter& coordsConverter;
};

VehicleDriver::VehicleDriver(const CoordsConverter& _coordsConverter)
    : coordsConverter(_coordsConverter) {
}

class StandardVehicleDriver : public VehicleDriver {

 public:
  StandardVehicleDriver(const CoordsConverter& coordsConverter);
  void driveVehicle(Vehicle& vehicle, const EgoCar& egoCar, double dt);
};

StandardVehicleDriver::StandardVehicleDriver(
    const CoordsConverter& coordsConverter)
    : VehicleDriver(coordsConverter) {
}

void StandardVehicleDriver::driveVehicle(Vehicle& vehicle, const EgoCar& egoCar,
                                         double dt) {
  vehicle.setPos(
      FrenetCart(
          vehicle.getPos().getFrenet() + (vehicle.getVel_frenet_m_per_s() * dt),
          coordsConverter));
// GTEST_COUT<< "vehicle: " << vehicle.getPos_frenet() << endl;
}

#endif /* TESTS_VEHICLEDRIVER_H_ */
