#include "../coords/coordsConverter.h"

#include <gtest/gtest.h>
#include <gtest/gtest-message.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../coords/cart.h"
#include "../coords/frenet.h"
#include "../coords/waypoints.h"
#include "../parametricSpline.h"

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

TEST(CoordsConverterTest, should_convert) {
  // GIVEN
  MapWaypoints map_waypoints;

  map_waypoints.map_waypoints.push_back(Point { 0, 10 });
  map_waypoints.map_outwards.push_back(Point { -1, 1 });

  map_waypoints.map_waypoints.push_back(Point { 0, 5 });
  map_waypoints.map_outwards.push_back(Point { -1, 0 });

  map_waypoints.map_waypoints.push_back(Point { 5, 0 });
  map_waypoints.map_outwards.push_back(Point { -1, -1 });

  map_waypoints.map_waypoints.push_back(Point { 10, 0 });
  map_waypoints.map_outwards.push_back(Point { 1, -1 });

  map_waypoints.map_waypoints.push_back(Point { 10, 10 });
  map_waypoints.map_outwards.push_back(Point { 1, 1 });

  for (int i = 0; i < map_waypoints.map_waypoints.size(); i++) {
    map_waypoints.map_waypoints_s.push_back(
        map_waypoints.getDistanceFromWaypointZeroToWaypoint(i));
  }

  CoordsConverter coordsConverter(map_waypoints);

  auto test_convert = [&](const Point& point, const Frenet& frenet) {
    const double abs_error = 0.00001;
    expect_near(frenet, coordsConverter.getFrenet(point), abs_error);
    expect_near(point, coordsConverter.getXY(frenet), abs_error);
  };

  // WHEN & THEN
  double s1 = 5;
  double s2 = sqrt(50);

  test_convert(Point { 0, 4 }, Frenet { s1 + 1 / sqrt(2), 1 / sqrt(2) });
  test_convert(Point { 4, 0 }, Frenet { s1 + s2 - 1 / sqrt(2), 1 / sqrt(2) });
  test_convert(Point { 9, 0.5 }, Frenet { s1 + s2 + 4, -0.5 });
}

void print_array(string name, vector<double> xs) {
  GTEST_COUT<< name << " = [";
  for (int i = 0; i < xs.size(); i++) {
    GTEST_COUT<< xs[i] << ", " << endl;
  }
  GTEST_COUT<< "]";
}

TEST(CoordsConverterTest, should_convert2) {
  MapWaypoints mapWaypoints = MapWaypoints::load();
  ParametricSpline spline(mapWaypoints.map_waypoints, SplineType::CatmullRom,
                          ParameterizationType::uniform);
  double x;
  double y;
  vector<double> xs;
  vector<double> ys;

  for (double t = 0.0; t < 1.0; t += 0.01) {
    Point p = spline(t);
    xs.push_back(p.x);
    ys.push_back(p.y);
  }

  print_array("x", xs);
  GTEST_COUT<< endl;
  print_array("y", ys);

  EXPECT_EQ(6947, int(spline.length()));
}

TEST(CoordsConverterTest, should_convert3) {
  // GIVEN
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);

  // TODO: DRY with should_convert
  auto test_convert = [&](const Point& point, const Frenet& frenet) {
    const double abs_error = 2.5;
    // TODO: die folgende Zeile wierder aktivieren, um getFrenet() zu entwickeln.
    // expect_near(frenet, coordsConverter.getFrenet(point), abs_error);
      expect_near(point, coordsConverter.getXY(frenet), abs_error, asString([&](stringstream& stream) {stream << point << " == coordsConverter.getXY(" << frenet <<")";}));
    };

  // WHEN & THEN
  test_convert(Point { 909.48, 1128.67 }, Frenet { 124.834, 6.16483 });
  for (int i = 0; i < mapWaypoints.map_waypoints.size(); i++) {
    test_convert(mapWaypoints.map_waypoints[i],
                 Frenet { mapWaypoints.map_waypoints_s[i], 0 });
  }

  vector<FrenetCart> positions = {
    FrenetCart(Frenet {124.834, 6.16483}, Point {909.48, 1128.67}),
    FrenetCart(Frenet {130.206, 6.12533}, Point {914.852, 1128.76}),
    FrenetCart(Frenet {132.214, 6.1577}, Point {916.86, 1128.74}),
    FrenetCart(Frenet {151.767, 6.07386}, Point {937.011, 1129.17}),
    FrenetCart(Frenet {177.5, 6.15793}, Point {962.601, 1131.88}),
    FrenetCart(Frenet {228.407, 6.30682}, Point {1013.16, 1145.99}),
    FrenetCart(Frenet {229.926, 6.26877}, Point {1014.55, 1146.61}),
    FrenetCart(Frenet {254.217, 5.98474}, Point {1036.82, 1156.16}),
    FrenetCart(Frenet {278.786, 5.98917}, Point {1059.61, 1165.49}),
    FrenetCart(Frenet {326.457, 5.98799}, Point {1103.2, 1179.97}),
    FrenetCart(Frenet {350.183, 5.96739}, Point {1126.72, 1183.04}),
    FrenetCart(Frenet {375.159, 5.90147}, Point {1151.3, 1185.03}),
    FrenetCart(Frenet {400.54, 5.97903}, Point {1176.47, 1186.41}),
    FrenetCart(Frenet {424.959, 5.97506}, Point {1200.86, 1187.54}),
    FrenetCart(Frenet {450.362, 5.83556}, Point {1226.16, 1188.63}),
    FrenetCart(Frenet {475.209, 5.8325}, Point {1250.62, 1188.48}),
    FrenetCart(Frenet {499.637, 5.97021}, Point {1274.95, 1187.52}),
    FrenetCart(Frenet {524.311, 5.8462}, Point {1299.6, 1186.53}),
    FrenetCart(Frenet {526.279, 5.87558}, Point {1301.57, 1186.41}),
    FrenetCart(Frenet {574.143, 6.03997}, Point {1348.89, 1181.47}),
    FrenetCart(Frenet {599.023, 5.97819}, Point {1373.64, 1179.23}),
    FrenetCart(Frenet {623.812, 5.99186}, Point {1398.11, 1176.82}),
    FrenetCart(Frenet {648.512, 5.99399}, Point {1422.6, 1173.61}),
    FrenetCart(Frenet {673.375, 6.00955}, Point {1447.29, 1170.45}),
    FrenetCart(Frenet {698.145, 6.00912}, Point {1471.86, 1167.35}),
    FrenetCart(Frenet {723.033, 6.0467}, Point {1496.7, 1164.27}),
    FrenetCart(Frenet {747.951, 6.02362}, Point {1521.65, 1161.81}),
    FrenetCart(Frenet {772.805, 5.99939}, Point {1546.35, 1159.94}),
    FrenetCart(Frenet {797.705, 6.00817}, Point {1571.14, 1157.67}),
    FrenetCart(Frenet {822.328, 5.9797}, Point {1595.89, 1156.13}),
    FrenetCart(Frenet {847.05, 5.93469}, Point {1620.37, 1154.05}),
    FrenetCart(Frenet {871.97, 6.08232}, Point {1644.88, 1150.99}),
    FrenetCart(Frenet {896.797, 6.00151}, Point {1669.99, 1149.15}),
    FrenetCart(Frenet {921.472, 5.99307}, Point {1694.46, 1147.21}),
    FrenetCart(Frenet {946.249, 6.05263}, Point {1719.22, 1145.15}),
    FrenetCart(Frenet {971.032, 6.02947}, Point {1744.06, 1143.5}),
    FrenetCart(Frenet {972.535, 6.02655}, Point {1745.56, 1143.42}),
    FrenetCart(Frenet {1020.39, 6.03753}, Point {1793.54, 1140.5}),
    FrenetCart(Frenet {1045.82, 6.09137}, Point {1818.97, 1139.92}),
    FrenetCart(Frenet {1071.03, 6.09302}, Point {1844.79, 1142.21}),
    FrenetCart(Frenet {1095.7, 6.22724}, Point {1869.48, 1145.19}),
    FrenetCart(Frenet {1119.94, 6.23602}, Point {1893.84, 1149.69}),
    FrenetCart(Frenet {1144.48, 6.21054}, Point {1917.94, 1155.61}),
    FrenetCart(Frenet {1169.47, 6.35835}, Point {1942.39, 1163.01}),
    FrenetCart(Frenet {1194.73, 6.40072}, Point {1966.59, 1173.16}),
    FrenetCart(Frenet {1219.82, 6.19452}, Point {1988.81, 1186.73}),
    FrenetCart(Frenet {1244.95, 6.0517}, Point {2009, 1203.4}),
    FrenetCart(Frenet {1246.49, 6.17113}, Point {2010.11, 1204.48}),
    FrenetCart(Frenet {1296.41, 6.32129}, Point {2040.3, 1245.34}),
    FrenetCart(Frenet {1321.41, 6.34835}, Point {2052.14, 1268}),
    FrenetCart(Frenet {1345.46, 6.20249}, Point {2062.05, 1290.47}),
    FrenetCart(Frenet {1370.14, 6.01752}, Point {2070.75, 1313.63}),
    FrenetCart(Frenet {1394.97, 6.12223}, Point {2079.55, 1336.84}),
    FrenetCart(Frenet {1397.5, 6.08824}, Point {2080.41, 1339.23}),
    FrenetCart(Frenet {1444.83, 6.05619}, Point {2094.48, 1384.88}),
    FrenetCart(Frenet {1468.83, 6.11284}, Point {2101.17, 1408.02}),
    FrenetCart(Frenet {1493.62, 6.02719}, Point {2107.79, 1432.64}),
    FrenetCart(Frenet {1519.29, 6.12766}, Point {2111.78, 1457.99}),
    FrenetCart(Frenet {1543.39, 5.95412}, Point {2114.96, 1482.01}),
    FrenetCart(Frenet {1567.81, 5.89931}, Point {2118.41, 1505.95}),
    FrenetCart(Frenet {1592.28, 5.93361}, Point {2122.62, 1530.05}),
    FrenetCart(Frenet {1617.41, 5.963}, Point {2127.2, 1554.65}),
    FrenetCart(Frenet {1641.52, 6.01098}, Point {2131.75, 1578.34}),
    FrenetCart(Frenet {1666.25, 5.86693}, Point {2136.11, 1602.72}),
    FrenetCart(Frenet {1690.59, 5.81214}, Point {2140.43, 1626.67}),
    FrenetCart(Frenet {1715.26, 5.83686}, Point {2145.72, 1650.45}),
    FrenetCart(Frenet {1739.71, 5.97222}, Point {2151.52, 1674.2}),
    FrenetCart(Frenet {1764.3, 6.17988}, Point {2157.43, 1698.06}),
    FrenetCart(Frenet {1789.09, 6.04545}, Point {2163.06, 1722.21}),
    FrenetCart(Frenet {1790.6, 6.01626}, Point {2163.38, 1723.69}),
    FrenetCart(Frenet {1838.19, 6.04255}, Point {2171.33, 1771.15}),
    FrenetCart(Frenet {1863.31, 6.02872}, Point {2174.95, 1796.01}),
    FrenetCart(Frenet {1887.73, 6.07423}, Point {2178.51, 1820.18}),
    FrenetCart(Frenet {1889.23, 6.08596}, Point {2178.74, 1821.66}),
    FrenetCart(Frenet {1937.35, 6.05911}, Point {2184.65, 1869.74}),
    FrenetCart(Frenet {1961.7, 5.88216}, Point {2186.84, 1893.73}),
    FrenetCart(Frenet {1963.19, 5.86329}, Point {2187.01, 1895.21}),
    FrenetCart(Frenet {2009.57, 5.88459}, Point {2194.19, 1940.74}),
    FrenetCart(Frenet {2034.01, 5.99835}, Point {2199.12, 1964.46}),
    FrenetCart(Frenet {2059.05, 6.07149}, Point {2204.19, 1989.17}),
    FrenetCart(Frenet {2083.03, 5.99995}, Point {2208.47, 2012.76}),
    FrenetCart(Frenet {2107.93, 5.8991}, Point {2212.95, 2037.24}),
    FrenetCart(Frenet {2131.99, 5.77085}, Point {2217.99, 2060.49}),
    FrenetCart(Frenet {2156.17, 5.86671}, Point {2223.88, 2083.53}),
    FrenetCart(Frenet {2180.56, 6.00136}, Point {2231.15, 2106.83}),
    FrenetCart(Frenet {2205.61, 6.107}, Point {2238.5, 2130.78}),
    FrenetCart(Frenet {2229.57, 6.15697}, Point {2244.76, 2154.21}),
    FrenetCart(Frenet {2254.02, 6.07169}, Point {2250.29, 2178.26}),
    FrenetCart(Frenet {2278.37, 6.05899}, Point {2255.27, 2202.11}),
    FrenetCart(Frenet {2303.03, 6.06321}, Point {2260.17, 2226.55}),
    FrenetCart(Frenet {2327.34, 5.97624}, Point {2263.96, 2250.56}),
    FrenetCart(Frenet {2352.26, 5.94684}, Point {2268.72, 2274.8}),
    FrenetCart(Frenet {2376.41, 5.71384}, Point {2273.38, 2298.43}),
    FrenetCart(Frenet {2400.68, 5.72312}, Point {2279.55, 2321.09}),
    FrenetCart(Frenet {2425.11, 6.0025}, Point {2287.88, 2344.17}),
    FrenetCart(Frenet {2450.09, 6.01722}, Point {2295.71, 2367.9}),
    FrenetCart(Frenet {2474.32, 6.17507}, Point {2303.42, 2390.87}),
    FrenetCart(Frenet {2498.48, 6.16466}, Point {2310.71, 2414.49}),
    FrenetCart(Frenet {2523.24, 6.09429}, Point {2316.16, 2438.64}),
    FrenetCart(Frenet {2548.1, 6.04995}, Point {2321.07, 2463.22}),
    FrenetCart(Frenet {2571.94, 6.023}, Point {2325.35, 2486.76}),
    FrenetCart(Frenet {2596.67, 6.04875}, Point {2329.75, 2511.09}),
    FrenetCart(Frenet {2620.83, 6.14881}, Point {2334.12, 2534.84}),
    FrenetCart(Frenet {2645.22, 6.23457}, Point {2336.98, 2559.59}),
    FrenetCart(Frenet {2669.72, 6.05036}, Point {2338.1, 2584.5}),
    FrenetCart(Frenet {2693.88, 5.97694}, Point {2337.81, 2608.84}),
    FrenetCart(Frenet {2718.22, 6.00433}, Point {2338.03, 2633.04}),
    FrenetCart(Frenet {2720.23, 6.00712}, Point {2338.06, 2635.05}),
    FrenetCart(Frenet {2767.21, 6.0112}, Point {2338.88, 2681.89}),
    FrenetCart(Frenet {2790.99, 6.0012}, Point {2339.07, 2705.82}),
    FrenetCart(Frenet {2815.34, 6.01023}, Point {2339.3, 2730.24}),
    FrenetCart(Frenet {2839.38, 6.03719}, Point {2339.28, 2754.2}),
    FrenetCart(Frenet {2864.11, 6.27814}, Point {2338.95, 2779.66}),
    FrenetCart(Frenet {2888.98, 6.26165}, Point {2335.23, 2805.01}),
    FrenetCart(Frenet {2913.65, 6.21949}, Point {2328.6, 2829.33}),
    FrenetCart(Frenet {2938.25, 6.18856}, Point {2319.98, 2853.01}),
    FrenetCart(Frenet {2963.9, 6.01927}, Point {2309.14, 2876.26}),
    FrenetCart(Frenet {2987.57, 6.29212}, Point {2296.9, 2897.3}),
    FrenetCart(Frenet {3011.54, 6.43769}, Point {2283.54, 2917.74}),
    FrenetCart(Frenet {3036.24, 6.33747}, Point {2267.91, 2938.15}),
    FrenetCart(Frenet {3060.8, 6.04711}, Point {2249.97, 2955.69}),
    FrenetCart(Frenet {3085.99, 6.53588}, Point {2231.04, 2972.32}),
    FrenetCart(Frenet {3109.85, 7.0228}, Point {2209.59, 2985.65}),
    FrenetCart(Frenet {3134.65, 6.62016}, Point {2185.26, 2995.64}),
    FrenetCart(Frenet {3160.6, 6.2392}, Point {2159.9, 3001.14}),
    FrenetCart(Frenet {3184.64, 6.39063}, Point {2135.11, 3003.83}),
    FrenetCart(Frenet {3208.79, 6.14688}, Point {2110.54, 3004.75}),
    FrenetCart(Frenet {3232.46, 5.98327}, Point {2086.91, 3004.84}),
    FrenetCart(Frenet {3256.63, 5.99801}, Point {2062.79, 3005.21}),
    FrenetCart(Frenet {3280.96, 6.14741}, Point {2038.47, 3005.94}),
    FrenetCart(Frenet {3305.14, 6.21455}, Point {2013.86, 3005.24}),
    FrenetCart(Frenet {3329.31, 6.05032}, Point {1989.55, 3003.63}),
    FrenetCart(Frenet {3353.33, 5.96469}, Point {1965.62, 3001.58}),
    FrenetCart(Frenet {3354.81, 5.98586}, Point {1964.15, 3001.48}),
    FrenetCart(Frenet {3401.91, 6.18824}, Point {1917.04, 2998.99}),
    FrenetCart(Frenet {3426.21, 6.09757}, Point {1892.58, 2996.36}),
    FrenetCart(Frenet {3450.67, 6.08726}, Point {1868.37, 2992.84}),
    FrenetCart(Frenet {3474.88, 5.90121}, Point {1844.33, 2988.29}),
    FrenetCart(Frenet {3499.58, 5.78833}, Point {1820.39, 2984.3}),
    FrenetCart(Frenet {3523.46, 5.92126}, Point {1796.66, 2981.66}),
    FrenetCart(Frenet {3547.08, 5.99988}, Point {1773.32, 2979.64}),
    FrenetCart(Frenet {3571.29, 6.05521}, Point {1748.97, 2977.56}),
    FrenetCart(Frenet {3595.64, 6.14154}, Point {1724.8, 2974.7}),
    FrenetCart(Frenet {3619.73, 6.09388}, Point {1700.72, 2971.09}),
    FrenetCart(Frenet {3643.83, 5.99386}, Point {1677, 2967.12}),
    FrenetCart(Frenet {3667.85, 5.86334}, Point {1653.28, 2963.32}),
    FrenetCart(Frenet {3691.92, 5.91252}, Point {1629.67, 2960.34}),
    FrenetCart(Frenet {3716.05, 6.05316}, Point {1605.66, 2957.84}),
    FrenetCart(Frenet {3717.56, 6.05027}, Point {1604.16, 2957.68}),
    FrenetCart(Frenet {3764.39, 5.59138}, Point {1557.65, 2951.11}),
    FrenetCart(Frenet {3789.36, 5.78288}, Point {1533.43, 2948.52}),
    FrenetCart(Frenet {3812.54, 5.54658}, Point {1510.27, 2947.59}),
    FrenetCart(Frenet {3837.02, 5.9441}, Point {1485.8, 2947.26}),
    FrenetCart(Frenet {3860.7, 5.87365}, Point {1462.23, 2946.84}),
    FrenetCart(Frenet {3884.64, 5.91009}, Point {1438.29, 2946.61}),
    FrenetCart(Frenet {3908.79, 5.85579}, Point {1414.3, 2946.67}),
    FrenetCart(Frenet {3932.65, 5.93218}, Point {1390.44, 2947.1}),
    FrenetCart(Frenet {3934.14, 5.95058}, Point {1388.95, 2947.14}),
    FrenetCart(Frenet {3980.82, 5.59807}, Point {1342.45, 2948.65}),
    FrenetCart(Frenet {4005.14, 5.81253}, Point {1318.71, 2950.3}),
    FrenetCart(Frenet {4028.77, 5.9528}, Point {1295.29, 2953.48}),
    FrenetCart(Frenet {4052.57, 6.06996}, Point {1271.34, 2956.56}),
    FrenetCart(Frenet {4076.76, 6.42556}, Point {1247.24, 2958.62}),
    FrenetCart(Frenet {4100.39, 6.03451}, Point {1223.21, 2959.84}),
    FrenetCart(Frenet {4125.03, 6.39544}, Point {1198.56, 2960.19}),
    FrenetCart(Frenet {4148.73, 6.20285}, Point {1174.87, 2959.98}),
    FrenetCart(Frenet {4172.46, 6.29167}, Point {1150.82, 2959.39}),
    FrenetCart(Frenet {4196.58, 6.00507}, Point {1126.34, 2957.77}),
    FrenetCart(Frenet {4220.62, 6.58601}, Point {1102.41, 2955.42}),
    FrenetCart(Frenet {4222.14, 6.60215}, Point {1100.9, 2955.25}),
    FrenetCart(Frenet {4268.76, 6.60496}, Point {1054.31, 2947.05}),
    FrenetCart(Frenet {4270.78, 6.62296}, Point {1052.34, 2946.59}),
    FrenetCart(Frenet {4316.9, 6.04745}, Point {1007.36, 2935.01}),
    FrenetCart(Frenet {4341.21, 5.94494}, Point {984.106, 2927.93}),
    FrenetCart(Frenet {4364.79, 5.81122}, Point {961.557, 2921.03}),
    FrenetCart(Frenet {4366.26, 5.84402}, Point {960.142, 2920.64}),
    FrenetCart(Frenet {4412.53, 5.85748}, Point {915.847, 2912.21}),
    FrenetCart(Frenet {4436.16, 5.61261}, Point {892.351, 2909.72}),
    FrenetCart(Frenet {4437.65, 5.62639}, Point {890.867, 2909.59}),
    FrenetCart(Frenet {4484.26, 5.54374}, Point {844.655, 2906.46}),
    FrenetCart(Frenet {4507.88, 5.65346}, Point {821.054, 2905.49}),
    FrenetCart(Frenet {4531.92, 5.66513}, Point {797.549, 2905.51}),
    FrenetCart(Frenet {4555.6, 5.926}, Point {773.913, 2906.84}),
    FrenetCart(Frenet {4579.42, 6.11794}, Point {750.025, 2907.95}),
    FrenetCart(Frenet {4603.44, 6.30823}, Point {726.027, 2908.88}),
    FrenetCart(Frenet {4627.02, 6.1941}, Point {702.015, 2908.98}),
    FrenetCart(Frenet {4651.09, 6.31734}, Point {677.965, 2908.16}),
    FrenetCart(Frenet {4675.07, 6.03687}, Point {654.014, 2906.93}),
    FrenetCart(Frenet {4698.83, 6.39022}, Point {630.094, 2905.66}),
    FrenetCart(Frenet {4722.84, 6.27408}, Point {605.609, 2902.95}),
    FrenetCart(Frenet {4725.36, 6.33681}, Point {603.111, 2902.57}),
    FrenetCart(Frenet {4770.27, 6.0881}, Point {558.581, 2894.26}),
    FrenetCart(Frenet {4794.3, 6.45613}, Point {535.156, 2888.93}),
    FrenetCart(Frenet {4818.57, 6.01558}, Point {511.675, 2882.75}),
    FrenetCart(Frenet {4819.58, 6.03776}, Point {510.214, 2882.33}),
    FrenetCart(Frenet {4867, 6.12458}, Point {465.006, 2866.99}),
    FrenetCart(Frenet {4889.89, 6.73542}, Point {443.5, 2859.14}),
    FrenetCart(Frenet {4914.5, 6.35846}, Point {420.755, 2849.73}),
    FrenetCart(Frenet {4937.79, 7.06135}, Point {399.246, 2838.01}),
    FrenetCart(Frenet {4961.77, 6.04371}, Point {378.866, 2823.53}),
    FrenetCart(Frenet {4985.95, 7.00373}, Point {360.347, 2807.94}),
    FrenetCart(Frenet {5010.7, 6.13644}, Point {342.647, 2790.62}),
    FrenetCart(Frenet {5033.69, 7.07183}, Point {326.702, 2772.74}),
    FrenetCart(Frenet {5035.73, 7.11815}, Point {325.39, 2771.19}),
    FrenetCart(Frenet {5082.08, 6.67188}, Point {297.59, 2732.78}),
    FrenetCart(Frenet {5106.28, 6.88111}, Point {285.772, 2711.67}),
    FrenetCart(Frenet {5130.38, 6.16459}, Point {274.813, 2690.19}),
    FrenetCart(Frenet {5153.72, 6.51273}, Point {264.233, 2668.9}),
    FrenetCart(Frenet {5178.39, 6.43436}, Point {253.983, 2646.47}),
    FrenetCart(Frenet {5201.77, 6.36109}, Point {245.133, 2624.16}),
    FrenetCart(Frenet {5225.27, 6.19967}, Point {237.646, 2601.89}),
    FrenetCart(Frenet {5248.89, 5.9948}, Point {230.148, 2579.52}),
    FrenetCart(Frenet {5273.07, 6.06643}, Point {222.108, 2556.71}),
    FrenetCart(Frenet {5296.66, 6.00424}, Point {214.353, 2534.29}),
    FrenetCart(Frenet {5320.87, 6.22725}, Point {206.666, 2511.34}),
    FrenetCart(Frenet {5343.77, 6.15119}, Point {199.666, 2489.53}),
    FrenetCart(Frenet {5367.27, 6.24315}, Point {192.947, 2466.7}),
    FrenetCart(Frenet {5391.12, 6.21868}, Point {186.697, 2443.69}),
    FrenetCart(Frenet {5414.51, 6.13501}, Point {180.801, 2420.82}),
    FrenetCart(Frenet {5438.46, 6.31513}, Point {175.196, 2397.53}),
    FrenetCart(Frenet {5462.35, 6.30803}, Point {170.608, 2373.61}),
    FrenetCart(Frenet {5485.8, 6.29519}, Point {167.125, 2350.41}),
    FrenetCart(Frenet {5509.68, 6.22559}, Point {164.162, 2326.37}),
    FrenetCart(Frenet {5533.6, 6.21074}, Point {161.961, 2302.55}),
    FrenetCart(Frenet {5557.13, 6.15135}, Point {160.453, 2278.78}),
    FrenetCart(Frenet {5580.36, 6.02859}, Point {159.543, 2255.57}),
    FrenetCart(Frenet {5604.39, 6.0111}, Point {158.403, 2231.61}),
    FrenetCart(Frenet {5628.07, 6.11665}, Point {157.057, 2207.97}),
    FrenetCart(Frenet {5651.04, 6.14831}, Point {156.174, 2184.8}),
    FrenetCart(Frenet {5674.8, 6.22793}, Point {155.702, 2161.04}),
    FrenetCart(Frenet {5699.12, 6.15798}, Point {155.685, 2136.47}),
    FrenetCart(Frenet {5721.82, 6.54153}, Point {155.85, 2113.78}),
    FrenetCart(Frenet {5745.96, 6.21112}, Point {156.763, 2089.65}),
    FrenetCart(Frenet {5748.49, 6.09925}, Point {156.937, 2087.12}),
    FrenetCart(Frenet {5792.94, 6.01241}, Point {163.728, 2042.18}),
    FrenetCart(Frenet {5817.06, 6.3463}, Point {167.988, 2018.44}),
    FrenetCart(Frenet {5841.54, 6.09791}, Point {172.889, 1994.45}),
    FrenetCart(Frenet {5844.07, 6.01265}, Point {173.454, 1991.98}),
    FrenetCart(Frenet {5888.04, 6.49534}, Point {184.678, 1949}),
    FrenetCart(Frenet {5911.06, 6.65096}, Point {193.352, 1926.61}),
    FrenetCart(Frenet {5935, 6.32092}, Point {203.609, 1904.97}),
    FrenetCart(Frenet {5958.38, 6.0495}, Point {213.756, 1883.78}),
    FrenetCart(Frenet {5981.97, 5.99508}, Point {224.025, 1862.54}),
    FrenetCart(Frenet {6005.58, 5.9336}, Point {234.198, 1841.3}),
    FrenetCart(Frenet {6029.09, 5.78503}, Point {244.328, 1820.08}),
    FrenetCart(Frenet {6052.87, 5.99814}, Point {254.126, 1798.72}),
    FrenetCart(Frenet {6076.25, 5.73385}, Point {263.32, 1777.22}),
    FrenetCart(Frenet {6099.63, 5.86429}, Point {272.148, 1755.57}),
    FrenetCart(Frenet {6123.2, 5.91894}, Point {280.775, 1733.79}),
    FrenetCart(Frenet {6146.64, 6.00489}, Point {289.103, 1711.89}),
    FrenetCart(Frenet {6170.02, 6.02055}, Point {297.552, 1689.96}),
    FrenetCart(Frenet {6194.08, 6.25673}, Point {306.406, 1667.59}),
    FrenetCart(Frenet {6217.3, 6.14513}, Point {315.744, 1645.89}),
    FrenetCart(Frenet {6240.53, 6.23689}, Point {325.834, 1624.96}),
    FrenetCart(Frenet {6263.85, 6.09006}, Point {336.489, 1604}),
    FrenetCart(Frenet {6287.41, 6.1161}, Point {347.481, 1583.15}),
    FrenetCart(Frenet {6310.77, 6.01348}, Point {358.538, 1562.51}),
    FrenetCart(Frenet {6335.24, 6.17321}, Point {370.066, 1540.92}),
    FrenetCart(Frenet {6357.51, 6.01594}, Point {380.972, 1521.23}),
    FrenetCart(Frenet {6381.1, 6.11594}, Point {392.947, 1500.91}),
    FrenetCart(Frenet {6404.54, 5.98967}, Point {405.042, 1480.83}),
    FrenetCart(Frenet {6406.04, 5.98627}, Point {405.809, 1479.54}),
    FrenetCart(Frenet {6451.87, 5.87085}, Point {428.924, 1440.06}),
    FrenetCart(Frenet {6474.8, 5.93577}, Point {440.013, 1420.18}),
    FrenetCart(Frenet {6498.14, 6.09065}, Point {450.955, 1399.57}),
    FrenetCart(Frenet {6521.36, 6.02719}, Point {462.201, 1378.97}),
    FrenetCart(Frenet {6544.84, 6.3024}, Point {473.941, 1358.64}),
    FrenetCart(Frenet {6568.45, 6.17456}, Point {486.098, 1338.39}),
    FrenetCart(Frenet {6591.58, 6.27909}, Point {498.525, 1318.56}),
    FrenetCart(Frenet {6615.17, 6.38245}, Point {511.421, 1298.8}),
    FrenetCart(Frenet {6617.19, 6.36412}, Point {512.544, 1297.12}),
    FrenetCart(Frenet {6661.65, 6.74615}, Point {538.544, 1260.52}),
    FrenetCart(Frenet {6685.9, 6.08479}, Point {553.79, 1241.66}),
    FrenetCart(Frenet {6708.84, 6.74453}, Point {570.485, 1224.55}),
    FrenetCart(Frenet {6732.66, 6.11391}, Point {588.173, 1208.58}),
    FrenetCart(Frenet {6755.62, 6.67125}, Point {605.814, 1193.16}),
    FrenetCart(Frenet {6779.94, 6.21869}, Point {624.939, 1178.14}),
    FrenetCart(Frenet {6803.29, 6.90501}, Point {645.593, 1165.13}),
    FrenetCart(Frenet {6827, 6.09104}, Point {666.663, 1154.22}),
    FrenetCart(Frenet {6850.44, 6.93782}, Point {689.006, 1144.55}),
    FrenetCart(Frenet {6872.19, 6.0891}, Point {710.814, 1137.39}),
    FrenetCart(Frenet {6896.31, 6.78835}, Point {734.488, 1132.69}),
    FrenetCart(Frenet {6920.15, 6.30752}, Point {759.034, 1129.95}),
  }
  ;

  for (const FrenetCart& position : positions) {
    test_convert(position.getXY(coordsConverter),
                 position.getFrenet(coordsConverter));
  }
}
