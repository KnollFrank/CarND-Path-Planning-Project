#ifndef PARAMETRICSPLINE_H_
#define PARAMETRICSPLINE_H_

#include "alglib/ap.h"
#include "alglib/interpolation.h"
#include "coords/cart.h"

#include <boost/math/tools/roots.hpp>

#include <boost/math/special_functions/next.hpp> // For float_distance.
#include <tuple> // for std::tuple and std::make_tuple.
#include <boost/math/special_functions/cbrt.hpp> // For boost::math::cbrt.

#include <iostream>
#include <iomanip>
#include <limits>

using namespace alglib;

enum SplineType {
	CatmullRom = 1, Cubic = 2
};

enum ParameterizationType {
	uniform = 0, chordLength = 1, centripetal = 2
};

class ParametricSpline {

public:
	ParametricSpline(const vector<Point>& points);

	Point operator()(double t) const;
	Point getTangent(double t) const;
	double length() const;

private:
	real_2d_array as_real_2d_array(const vector<Point> &points) const;

public:
	// TODO: make private again
	pspline2interpolant spline;
};

double ParametricSpline::length() const {
	return pspline2arclength(spline, 0, 1);
}

Point ParametricSpline::getTangent(double t) const {
	double x;
	double y;
	pspline2tangent(spline, t, x, y);
	return Point { x, y };
}

Point ParametricSpline::operator()(double t) const {
	double x;
	double y;
	pspline2calc(spline, t, x, y);
	return Point { x, y };
}

real_2d_array ParametricSpline::as_real_2d_array(
		const vector<Point> &points) const {

	real_2d_array xy;
	xy.setlength(points.size(), 2);
	for (int row = 0; row < points.size(); row++) {
		xy(row, 0) = points[row].x;
		xy(row, 1) = points[row].y;
	}
	return xy;
}

ParametricSpline::ParametricSpline(const vector<Point>& points) {
	real_2d_array xy = as_real_2d_array(points);
	pspline2buildperiodic(xy, points.size(), SplineType::CatmullRom,
			ParameterizationType::chordLength, spline);
}

double evaluatePoly(const vector<double>& coeff, double x) {
	return coeff[0] + coeff[1] * x + coeff[2] * x * x + coeff[3] * x * x * x
			+ coeff[4] * x * x * x * x + coeff[5] * x * x * x * x * x
			+ coeff[6] * x * x * x * x * x * x;
}

struct DistancePrimeFunctor {

	DistancePrimeFunctor(const vector<double>& distancePrimeCoeffs) :
			a(distancePrimeCoeffs) {
	}

	std::pair<double, double> operator()(double x) {
		double fx = evaluatePoly(a, x);
		double dx = a[1] + 2 * a[2] * x + 3 * a[3] * x * x + 4 * a[4]
				+ x * x * x + 5 * a[5] * x * x * x * x
				+ 6 * a[6] * x * x * x * x * x;
		return std::make_pair(fx, dx);
	}

private:
	const vector<double>& a;
};

double distancePrimeRoot(const vector<double>& distancePrimeCoeffs,
		double length) {
	using namespace boost::math::tools;
	// double guess = -distancePrimeCoeffs[0] / distancePrimeCoeffs[1];
	double min = 0;
	double max = 30.0 / length;
	double guess = 11.0 / length;
	const int digits = std::numeric_limits<double>::digits;
	int get_digits = static_cast<int>(digits * 0.6);
	const boost::uintmax_t maxit = 20;
	boost::uintmax_t it = maxit;
	DistancePrimeFunctor functor = DistancePrimeFunctor(distancePrimeCoeffs);
	double result = newton_raphson_iterate(functor, guess, min, max,
			get_digits/*, it*/);
	pair<double, double> tmp = functor(result);
	return result;
}

vector<double> polySquared(const vector<double>& a) {
	vector<double> d(7);
	d[0] = a[0] * a[0];
	d[1] = 2 * a[1] * a[0];
	d[2] = 2 * a[2] * a[0] + a[1] * a[1];
	d[3] = 2 * a[3] * a[0] + 2 * a[2] * a[1];
	d[4] = 2 * a[3] * a[1] + a[2] * a[2];
	d[5] = 2 * a[3] * a[2];
	d[6] = a[3] * a[3];
	return d;
}

vector<double> getDistancePrimeCoeffs(const Point& point,
		const vector<double>& a, const vector<double>& b) {
	vector<double> d = polySquared(a);
	vector<double> e = polySquared(b);
	vector<double> distancePrime(6);
	distancePrime[0] = -2 * point.x * a[1] + d[1] - 2 * point.y * b[1] + e[1];
	distancePrime[1] = 2
			* (-2 * point.x * a[2] + d[2] - 2 * point.y * b[2] + e[2]);
	distancePrime[2] = 3
			* (-2 * point.x * a[3] + d[3] - 2 * point.y * b[3] + e[3]);
	distancePrime[3] = 4 * (d[4] + e[4]);
	distancePrime[4] = 5 * (d[5] + e[5]);
	distancePrime[5] = 6 * (d[6] + e[6]);
	return distancePrime;
}

vector<double> getDistanceCoeffs(const Point& point, const vector<double>& a,
		const vector<double>& b) {
	vector<double> d = polySquared(a);
	vector<double> e = polySquared(b);
	vector<double> distance(7);
	distance[0] = point.x * point.x + point.y * point.y - 2 * point.x * a[0]
			+ d[0] - 2 * point.y * b[0] + e[0];
	distance[1] = -2 * point.x * a[1] + d[1] - 2 * point.y * b[1] + e[1];
	distance[2] = -2 * point.x * a[2] + d[2] - 2 * point.y * b[2] + e[2];
	distance[3] = -2 * point.x * a[3] + d[3] - 2 * point.y * b[3] + e[3];
	distance[4] = d[4] + e[4];
	distance[5] = d[5] + e[5];
	distance[6] = d[6] + e[6];
	return distance;
}

double getSquaredDistance(double t, const Point& point, const vector<double>& a,
		const vector<double>& b) {
	vector<double> distanceCoeffs = getDistanceCoeffs(point, a, b);
	return evaluatePoly(distanceCoeffs, t);
}

void spline1dunpack2(alglib_impl::spline1dinterpolant &c, ae_int_t &n,
		real_2d_array &tbl, const xparams _xparams) {
	jmp_buf _break_jump;
	alglib_impl::ae_state _alglib_env_state;
	alglib_impl::ae_state_init(&_alglib_env_state);
	if (setjmp(_break_jump)) {
#if !defined(AE_NO_EXCEPTIONS)
		_ALGLIB_CPP_EXCEPTION(_alglib_env_state.error_msg);
#else
		_ALGLIB_SET_ERROR_FLAG(_alglib_env_state.error_msg);
		return;
#endif
	}
	ae_state_set_break_jump(&_alglib_env_state, &_break_jump);
	if (_xparams.flags != 0x0)
		ae_state_set_flags(&_alglib_env_state, _xparams.flags);
	alglib_impl::spline1dunpack(&c, &n,
			const_cast<alglib_impl::ae_matrix*>(tbl.c_ptr()),
			&_alglib_env_state);
	alglib_impl::ae_state_clear(&_alglib_env_state);
	return;
}

double distance(const Point& point, const ParametricSpline& spline) {
	vector<double> a(4);
	vector<double> b(4);

	const pspline2interpolant &p1 = spline.spline;
	alglib_impl::pspline2interpolant* p =
			const_cast<alglib_impl::pspline2interpolant*>(p1.c_ptr());
	alglib_impl::spline1dinterpolant* c = &p->x;
	ae_int_t n;
	real_2d_array tbl;
	xparams _xparams;
	spline1dunpack2(p->x, n, tbl, _xparams);
	a[0] = tbl(0, 2);
	a[1] = tbl(0, 3);
	a[2] = tbl(0, 4);
	a[3] = tbl(0, 5);

	spline1dunpack2(p->y, n, tbl, _xparams);
	alglib_impl::spline1dinterpolant* d = &p->y;
	// double tmp = tbl(1, 0);
	b[0] = tbl(0, 2);
	b[1] = tbl(0, 3);
	b[2] = tbl(0, 4);
	b[3] = tbl(0, 5);

	vector<double> distancePrime = getDistancePrimeCoeffs(point, a, b);
	double length = spline.length();
	double root = distancePrimeRoot(distancePrime, length);
	double dist1 = sqrt(getSquaredDistance(root, point, a, b));
	double dist2 = sqrt(getSquaredDistance(c->x.ptr.p_double[0], point, a, b));
	double dist3 = sqrt(getSquaredDistance(c->x.ptr.p_double[1], point, a, b));
	double bla0 = c->x.ptr.p_double[0] * length;
	double bla1 = c->x.ptr.p_double[1] * length;
	double bla2 = c->x.ptr.p_double[2] * length;
	return dist1;
}

#endif /* PARAMETRICSPLINE_H_ */
