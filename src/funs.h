#ifndef FUNS_H_
#define FUNS_H_

#include <cmath>
#include <iostream>
#include <iterator>
#include <vector>
#include <boost/circular_buffer.hpp>

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() {
  return M_PI;
}

double deg2rad(double x) {
  return x * pi() / 180;
}

double rad2deg(double x) {
  return x * 180 / pi();
}

double mph2meter_per_sec(double t) {
  return t / 2.24;
}

double meter_per_sec2mph(double t) {
  return t * 2.24;
}

int sgn(double n) {
  return n >= 0 ? +1 : -1;
}

int modulo(int n, int N) {
  return n >= 0 ? n % N : N - ((-n) % N);
}

template<typename T, typename R, typename unop>
vector<R> map2(const vector<T> &v, unop op) {
  vector<R> result(v.size());
  transform(v.begin(), v.end(), result.begin(), op);
  return result;
}

template<typename T, typename unop>
void mapInPlace(vector<T> &v, unop op) {
  transform(v.begin(), v.end(), v.begin(), op);
}

template<typename T>
std::ostream& operator<<(std::ostream& out, const std::vector<T>& v) {
  if (!v.empty()) {
    out << '[';
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

// TODO: DRY with std::ostream& operator<<(std::ostream& out, const std::vector<T>& v)
template<typename T>
std::ostream& operator<<(std::ostream& out, const boost::circular_buffer<T>& v) {
  if (!v.empty()) {
    out << '[';
    std::copy(std::begin(v), std::end(v), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

template<typename T>
void appendSnd2Fst(vector<T>& fst, const vector<T>& snd) {
  fst.insert(std::end(fst), std::begin(snd), std::end(snd));
}

double getMinimum(const vector<double>& v) {
  return *std::min_element(v.begin(), v.end());
}

double getMaximum(const vector<double>& v) {
  return *std::max_element(v.begin(), v.end());
}

double getIndexOfMinimum(const vector<double>& v) {
  return std::distance(v.begin(), std::min_element(v.begin(), v.end()));
}

bool areNear(double a, double b, double EPSILON) {
  return fabs(a - b) < EPSILON;
}

#endif /* FUNS_H_ */
