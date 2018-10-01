#ifndef TESTS_GTESTHELPER_H_
#define TESTS_GTESTHELPER_H_

#include <gtest/gtest.h>
#include <gtest/gtest-message.h>
#include <functional>
#include <sstream>
#include <string>

#include "../coords/cart.h"
#include "../coords/frenet.h"

#define GTEST_COUT std::cerr

string asString(function<void(stringstream&)> print2Stream) {
	stringstream stream;
	print2Stream(stream);
	return stream.str();
}

void expect_near(const Frenet& expected, const Frenet& actual,
		const double abs_error, const string error_msg = "") {
	EXPECT_NEAR(expected.s, actual.s, abs_error) << error_msg;
	EXPECT_NEAR(expected.d, actual.d, abs_error) << error_msg;
}

void expect_near(const Point& expected, const Point& actual,
		const double abs_error, const string error_msg = "") {
	EXPECT_NEAR(expected.x, actual.x, abs_error) << error_msg;
	EXPECT_NEAR(expected.y, actual.y, abs_error) << error_msg;
}

#endif /* TESTS_GTESTHELPER_H_ */
