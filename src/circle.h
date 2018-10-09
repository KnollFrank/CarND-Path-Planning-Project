#ifndef CIRCLE_H_
#define CIRCLE_H_

#include "coords/cart.h"

class Circle {

 public:
  Point center;
  double radius;

  bool overlaps(const Circle& other) {
    return center.distanceTo(other.center) <= radius + other.radius;
  }
};

#endif /* CIRCLE_H_ */
