#ifndef CAR_H_
#define CAR_H_

#include <iostream>

#include "coords/cart.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "dimension.h"
#include "funs.h"
#include "rectangle.h"
#include <boost/circular_buffer.hpp>

using namespace std;

enum PositionHistory {
  ACTUAL = 3,
  PREVIOUS = 2,
  PREVIOUS_PREVIOUS = 1,
  PREVIOUS_PREVIOUS_PREVIOUS = 0
};

// TODO: each class within this file shall be placed into it's own file
// TODO: merge EgoCar and Vehicle, because they are essentially the same thing.
class EgoCar {

 public:
  EgoCar(const CoordsConverter& coordsConverter);
  double yaw_deg;
  double speed_mph;

  void setPos(const FrenetCart& pos);
  FrenetCart getPos() const;
  Point getAcceleration(double dt) const;
  Point getJerk(double dt) const;
  Rectangle getShape() const;

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);

 private:
  Point getVelocity(const PositionHistory& positionHistory, double dt) const;
  Point getAcceleration(const PositionHistory& positionHistory,
                        double dt) const;

  const CoordsConverter& coordsConverter;
  boost::circular_buffer<FrenetCart> positions;
  const Dimension shapeTemplate;
};

ostream& operator<<(ostream& os, const EgoCar& egoCar) {
  os << "EgoCar:" << endl;
  os << "  position = " << egoCar.getPos();
  os << "  yaw = " << egoCar.yaw_deg << "°" << endl;
  os << "  speed = " << egoCar.speed_mph << " mph ("
     << mph2meter_per_sec(egoCar.speed_mph) << " m/s)" << endl;
  return os;
}

EgoCar::EgoCar(const CoordsConverter& _coordsConverter)
    : coordsConverter(_coordsConverter),
      positions(boost::circular_buffer<FrenetCart>(4)),
      shapeTemplate(Dimension::fromWidthAndHeight(2.5, 2.5)) {
}

// TODO: DRY with Vehicle.getShape()
Rectangle EgoCar::getShape() const {
  return Rectangle::fromCenterAndDimension(getPos().getFrenet(), shapeTemplate);
}

void EgoCar::setPos(const FrenetCart& pos) {
  positions.push_back(pos);
}

FrenetCart EgoCar::getPos() const {
  return positions.back();
}

Point EgoCar::getVelocity(const PositionHistory& positionHistory,
                          double dt) const {
  return (positions[positionHistory].getXY()
      - positions[positionHistory - 1].getXY()) / dt;
}

Point EgoCar::getAcceleration(const PositionHistory& positionHistory,
                              double dt) const {
  return
      positions.full() ?
          (getVelocity(positionHistory, dt)
              - getVelocity(static_cast<PositionHistory>(positionHistory - 1),
                            dt)) / dt :
          Point::zero();
}

Point EgoCar::getAcceleration(double dt) const {
  return getAcceleration(PositionHistory::ACTUAL, dt);
}

Point EgoCar::getJerk(double dt) const {
  return
      positions.full() ?
          (getAcceleration(PositionHistory::ACTUAL)
              - getAcceleration(PositionHistory::PREVIOUS)) / dt :
          Point::zero();
}

#endif /* CAR_H_ */
