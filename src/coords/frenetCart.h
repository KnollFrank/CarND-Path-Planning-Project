#ifndef COORDS_FRENETCART_H_
#define COORDS_FRENETCART_H_

#include <iostream>

#include "cart.h"
#include "coordsConverter.h"
#include "frenet.h"
#include <experimental/optional>

class FrenetCart {
 public:
  FrenetCart();
  FrenetCart(Frenet frenet, Point cart, const CoordsConverter& coordsConverter);
  FrenetCart(Frenet frenet, const CoordsConverter& _coordsConverter);
  FrenetCart(Point cart, const CoordsConverter& _coordsConverter);

  Frenet getFrenet() const;
  Point getXY() const;

  friend ostream& operator<<(ostream& os, const FrenetCart& frenetCart);

 private:
  const CoordsConverter* coordsConverter;
  std::experimental::optional<Frenet> frenet;
  std::experimental::optional<Point> cart;
};

ostream& operator<<(ostream& os, const FrenetCart& frenetCart) {
  os << "FrenetCart:" << endl;
  if (frenetCart.frenet) {
    os << "  frenet = " << *frenetCart.frenet << endl;
  }
  if (frenetCart.cart) {
    os << "  cart = " << *frenetCart.cart << endl;
  }
  return os;
}

FrenetCart::FrenetCart(Frenet _frenet, Point _cart,
                       const CoordsConverter& _coordsConverter)
    : frenet(_frenet),
      cart(_cart),
      coordsConverter(&_coordsConverter) {
}

FrenetCart::FrenetCart()
    : frenet(std::experimental::nullopt),
      cart(std::experimental::nullopt) {
}

FrenetCart::FrenetCart(Frenet _frenet, const CoordsConverter& _coordsConverter)
    : frenet(_frenet),
      cart(std::experimental::nullopt),
      coordsConverter(&_coordsConverter) {
}

FrenetCart::FrenetCart(Point _cart, const CoordsConverter& _coordsConverter)
    : frenet(std::experimental::nullopt),
      cart(_cart),
      coordsConverter(&_coordsConverter) {
}

Frenet FrenetCart::getFrenet() const {
  return frenet ? *frenet : coordsConverter->getFrenet(*cart);
}

Point FrenetCart::getXY() const {
  return cart ? *cart : coordsConverter->getXY(*frenet);
}

#endif /* COORDS_FRENETCART_H_ */
