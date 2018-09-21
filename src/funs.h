#ifndef FUNS_H_
#define FUNS_H_

#include <vector>
#include <math.h>

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
    std::copy(v.begin(), v.end(), std::ostream_iterator < T > (out, ", "));
    out << "\b\b]";
  }
  return out;
}

template<typename T>
void appendSnd2Fst(vector<T>& fst, const vector<T>& snd) {
  fst.insert(std::end(fst), std::begin(snd), std::end(snd));
}

#endif /* FUNS_H_ */
