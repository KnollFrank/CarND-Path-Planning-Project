#ifndef RECTANGLE_H_
#define RECTANGLE_H_

#include "coords/frenet.h"
#include "dimension.h"

class Rectangle {

 public:
  static Rectangle fromTopLeftAndDimension(const Frenet& topLeft,
                                           const Dimension& dimension);
  static Rectangle fromCenterAndDimension(const Frenet& center,
                                          const Dimension& dimension);

  Frenet getBottomRight() const {
    return topLeft + Frenet { -dimension.getHeight(), dimension.getWidth() };
  }

  Frenet getTopLeft() const {
    return topLeft;
  }

  double getWidth() const {
    return dimension.getWidth();
  }

  double getHeight() const {
    return dimension.getHeight();
  }

  bool overlaps(const Rectangle& other) const {
    return !this->isOnLeftSideOf(other) && !other.isOnLeftSideOf(*this)
        && !this->isAbove(other) && !other.isAbove(*this);
  }

 private:
  Rectangle(const Frenet& topLeft, const Dimension& dimension);

  bool isOnLeftSideOf(const Rectangle& other) const {
    return other.getTopLeft().d > getBottomRight().d;
  }

  bool isAbove(const Rectangle& other) const {
    return other.getTopLeft().s < getBottomRight().s;
  }

  const Frenet topLeft;
  const Dimension dimension;
};

Rectangle Rectangle::fromTopLeftAndDimension(const Frenet& topLeft,
                                             const Dimension& dimension) {
  return Rectangle(topLeft, dimension);
}

Rectangle Rectangle::fromCenterAndDimension(const Frenet& center,
                                            const Dimension& dimension) {
  return fromTopLeftAndDimension(
      center + Frenet { dimension.getHeight() / 2, -dimension.getWidth() / 2 },
      dimension);
}

Rectangle::Rectangle(const Frenet& _topLeft, const Dimension& _dimension)
    : topLeft(_topLeft),
      dimension(_dimension) {
}

#endif /* RECTANGLE_H_ */
