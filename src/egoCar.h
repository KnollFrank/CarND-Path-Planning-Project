#ifndef EGOCAR_H_
#define EGOCAR_H_

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

PositionHistory getPrevious(PositionHistory positionHistory) {
  switch (positionHistory) {
    case ACTUAL:
      return PREVIOUS;
    case PREVIOUS:
      return PREVIOUS_PREVIOUS;
    case PREVIOUS_PREVIOUS:
      return PREVIOUS_PREVIOUS_PREVIOUS;
  }
}

// TODO: merge EgoCar and Vehicle, because they are essentially the same thing.
class EgoCar {

 public:
  EgoCar(const CoordsConverter& coordsConverter, int considerOnlyEachNthPos);
  double yaw_deg;
  double speed_mph;

  void setPos(const FrenetCart& pos);
  FrenetCart getPos() const;
  Point getAcceleration(double dt) const;
  Point getVelocity(double dt) const;
  Point getJerk(double dt) const;
  Rectangle getShape() const;

  friend ostream& operator<<(ostream& os, const EgoCar& egoCar);

 private:
  int asIndex(const PositionHistory& positionHistory) const;
  Point getVelocity(const PositionHistory& positionHistory, double dt) const;
  Point getAcceleration(const PositionHistory& positionHistory,
                        double dt) const;
  double getDt(double dt) const;

  const CoordsConverter& coordsConverter;
  const int considerOnlyEachNthPos;
  const Dimension shapeTemplate;
 public:
  boost::circular_buffer<FrenetCart> positions;
};

ostream& operator<<(ostream& os, const EgoCar& egoCar) {
  os << "EgoCar:" << endl;
  os << "  position = " << egoCar.getPos();
  os << "  yaw = " << egoCar.yaw_deg << "°" << endl;
  os << "  speed = " << egoCar.speed_mph << " mph ("
     << mph2meter_per_sec(egoCar.speed_mph) << " m/s)" << endl;
  return os;
}

EgoCar::EgoCar(const CoordsConverter& _coordsConverter,
               int _considerOnlyEachNthPos)
    : coordsConverter(_coordsConverter),
      considerOnlyEachNthPos(_considerOnlyEachNthPos),
      positions(
          boost::circular_buffer<FrenetCart>(3 * _considerOnlyEachNthPos + 1)),
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

int EgoCar::asIndex(const PositionHistory& positionHistory) const {
  return positionHistory * considerOnlyEachNthPos;
}

Point EgoCar::getVelocity(const PositionHistory& positionHistory,
                          double dt) const {
  return (positions[asIndex(positionHistory)].getXY()
      - positions[asIndex(getPrevious(positionHistory))].getXY()) / getDt(dt);
}

double EgoCar::getDt(double dt) const {
  return considerOnlyEachNthPos * dt;
}

Point EgoCar::getAcceleration(const PositionHistory& positionHistory,
                              double dt) const {
  return (getVelocity(positionHistory, dt)
      - getVelocity(getPrevious(positionHistory), dt)) / getDt(dt);
}

Point EgoCar::getAcceleration(double dt) const {
  return
      positions.full() ?
          getAcceleration(PositionHistory::ACTUAL, dt) : Point::zero();
}

Point EgoCar::getVelocity(double dt) const {
  return
      positions.full() ?
          getVelocity(PositionHistory::ACTUAL, dt) : Point::zero();
}

Point EgoCar::getJerk(double dt) const {
  return
      positions.full() ?
          (getAcceleration(PositionHistory::ACTUAL)
              - getAcceleration(PositionHistory::PREVIOUS)) / getDt(dt) :
          Point::zero();
}

#endif /* EGOCAR_H_ */
