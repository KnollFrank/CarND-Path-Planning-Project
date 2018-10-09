#ifndef TESTS_VEHICLEDRIVER_H_
#define TESTS_VEHICLEDRIVER_H_

#include "../egoCar.h"
#include "../coords/frenet.h"
#include "../coords/frenetCart.h"

class VehicleDriver {

 public:
  VehicleDriver(const CoordsConverter& coordsConverter);

  virtual ~VehicleDriver() {
  }

  void driveVehicle(Vehicle& vehicle, const EgoCar& egoCar, double dt);

  virtual FrenetCart getNewPos(const Vehicle& vehicle, const EgoCar& egoCar,
                               double dt) = 0;

 protected:
  const CoordsConverter& coordsConverter;
};

VehicleDriver::VehicleDriver(const CoordsConverter& _coordsConverter)
    : coordsConverter(_coordsConverter) {
}

void VehicleDriver::driveVehicle(Vehicle& vehicle, const EgoCar& egoCar,
                                 double dt) {
  vehicle.setPos(getNewPos(vehicle, egoCar, dt));
// GTEST_COUT<< "vehicle: " << vehicle.getPos_frenet() << endl;
}

class StandardVehicleDriver : public VehicleDriver {

 public:
  StandardVehicleDriver(const CoordsConverter& coordsConverter);
  FrenetCart getNewPos(const Vehicle& vehicle, const EgoCar& egoCar, double dt)
      override;
};

StandardVehicleDriver::StandardVehicleDriver(
    const CoordsConverter& coordsConverter)
    : VehicleDriver(coordsConverter) {
}

FrenetCart StandardVehicleDriver::getNewPos(const Vehicle& vehicle,
                                            const EgoCar& egoCar, double dt) {
  return FrenetCart(
      vehicle.getPos().getFrenet() + (vehicle.getVel_frenet_m_per_s() * dt),
      coordsConverter);
}

#endif /* TESTS_VEHICLEDRIVER_H_ */
