#ifndef COORDS_FRENETCART_H_
#define COORDS_FRENETCART_H_

#include <iostream>

#include "cart.h"
#include "coordsConverter.h"
#include "frenet.h"

class FrenetCart {
 public:
  FrenetCart();
  FrenetCart(const Frenet& frenet, const Point& cart,
             const CoordsConverter& coordsConverter);
  FrenetCart(const Frenet& frenet, const CoordsConverter& _coordsConverter);
  FrenetCart(const Point& cart, const CoordsConverter& coordsConverter);

  Frenet getFrenet() const;
  Point getXY() const;

  friend ostream& operator<<(ostream& os, const FrenetCart& frenetCart);

 private:
  const CoordsConverter* coordsConverter;
  Frenet frenet;
  Point cart;
};

ostream& operator<<(ostream& os, const FrenetCart& frenetCart) {
  os << "FrenetCart:" << endl;
  os << "  frenet = " << frenetCart.getFrenet() << endl;
  os << "  cart = " << frenetCart.getXY() << endl;
  return os;
}

FrenetCart::FrenetCart(const Frenet& _frenet, const Point& _cart,
                       const CoordsConverter& _coordsConverter)
    : frenet(_frenet),
      cart(_cart),
      coordsConverter(&_coordsConverter) {
}

FrenetCart::FrenetCart() {
}

FrenetCart::FrenetCart(const Frenet& _frenet,
                       const CoordsConverter& _coordsConverter)
    : frenet(_frenet),
      cart(_coordsConverter.getXY(_frenet)),
      coordsConverter(&_coordsConverter) {
}

FrenetCart::FrenetCart(const Point& _cart,
                       const CoordsConverter& _coordsConverter)
    : frenet(_coordsConverter.getFrenet(_cart)),
      cart(_cart),
      coordsConverter(&_coordsConverter) {
}

Frenet FrenetCart::getFrenet() const {
  return frenet;
}

Point FrenetCart::getXY() const {
  return cart;
}

#endif /* COORDS_FRENETCART_H_ */
