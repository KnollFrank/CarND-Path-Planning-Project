#ifndef COORDS_FRENETCART_H_
#define COORDS_FRENETCART_H_

#include "cart.h"
#include "frenet.h"
#include "coordsConverter.h"
#include <experimental/optional>

class FrenetCart {
 public:
  FrenetCart();
  FrenetCart(Frenet frenet, Point cart);
  FrenetCart(Frenet frenet);
  FrenetCart(Point cart);

  Frenet getFrenet(const CoordsConverter& coordsConverter) const;
  Point getXY(const CoordsConverter& coordsConverter) const;

  friend ostream& operator<<(ostream& os, const FrenetCart& frenetCart);

 private:
  std::experimental::optional<Frenet> frenet;
  std::experimental::optional<Point> cart;
};

ostream& operator<<(ostream& os, const FrenetCart& frenetCart) {
  os << "FrenetCart:" << endl;
  os << "  frenet = " << *frenetCart.frenet << endl;
  os << "  cart = " << *frenetCart.cart << endl;
  return os;
}

FrenetCart::FrenetCart(Frenet _frenet, Point _cart)
    : frenet(_frenet),
      cart(_cart) {
}

FrenetCart::FrenetCart()
    : frenet(std::experimental::nullopt),
      cart(std::experimental::nullopt) {
}

FrenetCart::FrenetCart(Frenet _frenet)
    : frenet(_frenet),
      cart(std::experimental::nullopt) {
}

FrenetCart::FrenetCart(Point _cart)
    : frenet(std::experimental::nullopt),
      cart(_cart) {
}

Frenet FrenetCart::getFrenet(const CoordsConverter& coordsConverter) const {
  return frenet ? frenet.value() : coordsConverter.getFrenet(cart.value());
}

Point FrenetCart::getXY(const CoordsConverter& coordsConverter) const {
  return cart ? cart.value() : coordsConverter.getXY(frenet.value());
}

#endif /* COORDS_FRENETCART_H_ */
