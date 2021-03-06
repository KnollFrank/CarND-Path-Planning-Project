#ifndef PREVIOUSDATA_H_
#define PREVIOUSDATA_H_

#include "json.hpp"
#include <map>
#include <string>
#include <vector>

#include "coords/cart.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "path.h"

using namespace std;

// for convenience
using json = nlohmann::json;

struct PreviousData {
  Path previous_path;
  FrenetCart end_path;

  int sizeOfPreviousPath() const;
  static PreviousData fromJson(
      const nlohmann::basic_json<std::map, std::vector,
          std::__cxx11::basic_string<char, std::char_traits<char>,
              std::allocator<char> >, bool, long, unsigned long, double,
          std::allocator, nlohmann::adl_serializer> &j,
      const CoordsConverter& coordsConverter);
};

int PreviousData::sizeOfPreviousPath() const {
  return previous_path.points.size();
}

template<typename T>
void print_array2(string name, vector<T> xs) {
  cout << name << " = [";
  for (int i = 0; i < xs.size(); i++) {
    cout << xs[i] << ", " << endl;
  }
  cout << "]";
}

PreviousData PreviousData::fromJson(
    const nlohmann::basic_json<std::map, std::vector,
        std::__cxx11::basic_string<char, std::char_traits<char>,
            std::allocator<char> >, bool, long, unsigned long, double,
        std::allocator, nlohmann::adl_serializer> &j,
    const CoordsConverter& coordsConverter) {
  PreviousData previousData;
  // Previous path data given to the Planner
  vector<double> previous_path_x = j[1]["previous_path_x"];
  vector<double> previous_path_y = j[1]["previous_path_y"];
  for (int i = 0; i < previous_path_x.size(); i++) {
    FrenetCart frenetCart = FrenetCart(Point { previous_path_x[i],
                                           previous_path_y[i] },
                                       coordsConverter);
    previousData.previous_path.points.push_back(frenetCart);
  }

// Previous path's end s and d values
  if (previousData.sizeOfPreviousPath() > 0) {
    previousData.end_path = previousData.previous_path.points.back();
  } else {
    previousData.end_path = FrenetCart(
        Frenet { coordsConverter.adapt_s_coord(j[1]["end_path_s"]),
            j[1]["end_path_d"] },
        coordsConverter);
  }
  return previousData;
}

ostream& operator<<(ostream& os, const PreviousData& previousData) {
  os << "PreviousData:" << endl;
  os << "  previous_path = " << previousData.previous_path << endl;
  os << "  end_path = " << previousData.end_path << endl;
  return os;
}

#endif /* PREVIOUSDATA_H_ */
