#ifndef LANE_H_
#define LANE_H_

#include "car.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include <experimental/optional>

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

bool isInLane(double d, const Lane& lane) {
  return startOfLane(lane) <= d && d <= endOfLane(lane);
}

bool isVehicleInLane(const Vehicle &vehicle, const Lane& lane) {
  return isInLane(vehicle.getPos().getFrenet().d, lane);
}

std::experimental::optional<Lane> getLane(double d) {
  for (const Lane& lane : { Lane::LEFT, Lane::MIDDLE, Lane::RIGHT }) {
    if (isInLane(d, lane)) {
      return std::experimental::make_optional(lane);
    }
  }

  return std::experimental::nullopt;
}

#endif /* LANE_H_ */
