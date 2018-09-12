#ifndef LANE_H_
#define LANE_H_

enum Lane {
  LEFT = 0,
  MIDDLE = 1,
  RIGHT = 2
};

double sizeOfLane() {
  return 4;
}

double startOfLane(int lane) {
  return sizeOfLane() * lane;
}

double getMiddleOfLane(Lane lane) {
  return startOfLane(lane) + sizeOfLane() / 2;
}

double endOfLane(Lane lane) {
  return startOfLane(lane + 1);
}

bool isInLane(float d, Lane lane) {
  return startOfLane(lane) < d && d < endOfLane(lane);
}

bool isVehicleInLane(const Vehicle &vehicle, Lane lane) {
  return isInLane(vehicle.getPos_frenet().d, lane);
}

#endif /* LANE_H_ */
