#ifndef PATH_H_
#define PATH_H_

#include <vector>
#include "coords/cart.h"
#include "json.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

struct Path {
  vector<Point> points;
};

#endif /* PATH_H_ */
