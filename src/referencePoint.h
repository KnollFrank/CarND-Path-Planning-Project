#ifndef REFERENCEPOINT_H_
#define REFERENCEPOINT_H_

#include "coords/frenet.h"

struct ReferencePoint {
  Frenet point;
  double yaw_rad;
  double vel_mph;
};

#endif /* REFERENCEPOINT_H_ */
