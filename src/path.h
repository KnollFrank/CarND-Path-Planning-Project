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

  tuple<vector<double>, vector<double>> asXValsAndYVals() const;
};

tuple<vector<double>, vector<double>> Path::asXValsAndYVals() const {
  vector<double> xs;
  vector<double> ys;
  for (const Point& point : points) {
    xs.push_back(point.x);
    ys.push_back(point.y);
  }

  return make_tuple(xs, ys);
}

#endif /* PATH_H_ */
