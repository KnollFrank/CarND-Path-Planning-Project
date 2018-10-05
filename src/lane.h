#ifndef LANE_H_
#define LANE_H_

#include "car.h"
#include "coords/frenet.h"

enum Lane {
  LEFT = 0,
  MIDDLE = 1,
  RIGHT = 2
};

double widthOfLane() {
  return 4;
}

double startOfLane(int lane) {
  return widthOfLane() * lane;
}

double getMiddleOfLane(const Lane& lane) {
  return startOfLane(lane) + widthOfLane() / 2;
}

double endOfLane(const Lane& lane) {
  return startOfLane(lane + 1);
}

bool isInLane(float d, const Lane& lane) {
  return startOfLane(lane) <= d && d <= endOfLane(lane);
}

bool isVehicleInLane(const Vehicle &vehicle, const Lane& lane) {
  return isInLane(vehicle.getPos().getFrenet().d, lane);
}

#endif /* LANE_H_ */
