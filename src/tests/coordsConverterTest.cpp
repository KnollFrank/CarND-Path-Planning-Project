#include "../coords/coordsConverter.h"

#include <gtest/gtest.h>
#include <gtest/gtest-message.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>

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

  vector<tuple<Frenet, Point>> frenetPointTuples = { make_tuple(Frenet {
                                                                    124.834,
                                                                    6.16483 },
                                                                Point { 909.48,
                                                                    1128.67 }),
      make_tuple(Frenet { 125.22, 6.04427 }, Point { 909.866, 1128.79 }),
      make_tuple(Frenet { 127.707, 6.08273 }, Point { 912.353, 1128.78 }),
      make_tuple(Frenet { 151.259, 6.0505 }, Point { 936.504, 1129.14 }),
      make_tuple(Frenet { 178.012, 6.13358 }, Point { 963.107, 1131.96 }),
      make_tuple(Frenet { 227.899, 6.3181 }, Point { 1012.7, 1145.78 }),
      make_tuple(Frenet { 252.721, 5.98308 }, Point { 1035.43, 1155.6 }),
      make_tuple(Frenet { 277.76, 5.99766 }, Point { 1058.66, 1165.08 }),
      make_tuple(Frenet { 301.836, 5.96451 }, Point { 1080.86, 1174.4 }),
      make_tuple(Frenet { 327.074, 5.95529 }, Point { 1103.81, 1180.08 }),
      make_tuple(Frenet { 328.997, 5.86922 }, Point { 1105.7, 1180.41 }),
      make_tuple(Frenet { 375.94, 5.90856 }, Point { 1152.08, 1185.08 }),
      make_tuple(Frenet { 377.435, 5.92276 }, Point { 1153.57, 1185.18 }),
      make_tuple(Frenet { 426.229, 5.97568 }, Point { 1202.13, 1187.6 }),
      make_tuple(Frenet { 451.114, 5.83688 }, Point { 1226.92, 1188.66 }),
      make_tuple(Frenet { 475.465, 5.83336 }, Point { 1250.87, 1188.47 }),
      make_tuple(Frenet { 500.399, 5.96283 }, Point { 1275.71, 1187.49 }),
      make_tuple(Frenet { 525.049, 5.85604 }, Point { 1300.34, 1186.49 }),
      make_tuple(Frenet { 526.525, 5.87954 }, Point { 1301.81, 1186.4 }),
      make_tuple(Frenet { 574.897, 6.03911 }, Point { 1349.64, 1181.4 }),
      make_tuple(Frenet { 599.775, 5.97329 }, Point { 1374.39, 1179.17 }),
      make_tuple(Frenet { 624.543, 5.98063 }, Point { 1398.83, 1176.74 }),
      make_tuple(Frenet { 649.76, 5.99854 }, Point { 1423.83, 1173.44 }),
      make_tuple(Frenet { 674.117, 6.00898 }, Point { 1448.03, 1170.36 }),
      make_tuple(Frenet { 698.893, 6.01187 }, Point { 1472.6, 1167.26 }),
      make_tuple(Frenet { 723.789, 6.05303 }, Point { 1497.46, 1164.19 }),
      make_tuple(Frenet { 748.722, 6.02875 }, Point { 1522.42, 1161.75 }),
      make_tuple(Frenet { 773.546, 5.99404 }, Point { 1547.09, 1159.88 }),
      make_tuple(Frenet { 798.818, 6.00678 }, Point { 1572.43, 1157.57 }),
      make_tuple(Frenet { 823.065, 5.98687 }, Point { 1596.63, 1156.08 }),
      make_tuple(Frenet { 847.787, 5.94239 }, Point { 1621.1, 1153.97 }),
      make_tuple(Frenet { 873.784, 6.06185 }, Point { 1646.68, 1150.78 }),
      make_tuple(Frenet { 897.549, 5.99533 }, Point { 1670.74, 1149.11 }),
      make_tuple(Frenet { 922.231, 5.99664 }, Point { 1695.22, 1147.14 }),
      make_tuple(Frenet { 947.011, 6.05278 }, Point { 1719.98, 1145.09 }),
      make_tuple(Frenet { 971.796, 6.02888 }, Point { 1744.82, 1143.46 }),
      make_tuple(Frenet { 996.927, 6.00731 }, Point { 1769.86, 1142 }),
      make_tuple(Frenet { 1021.19, 6.05601 }, Point { 1794.34, 1140.47 }),
      make_tuple(Frenet { 1047.19, 6.02287 }, Point { 1820.34, 1139.96 }),
      make_tuple(Frenet { 1071.36, 6.08611 }, Point { 1845.12, 1142.24 }),
      make_tuple(Frenet { 1096.05, 6.22429 }, Point { 1869.83, 1145.24 }),
      make_tuple(Frenet { 1120.8, 6.23005 }, Point { 1894.68, 1149.89 }),
      make_tuple(Frenet { 1145.36, 6.22014 }, Point { 1918.79, 1155.83 }),
      make_tuple(Frenet { 1170.88, 6.39479 }, Point { 1943.72, 1163.46 }),
      make_tuple(Frenet { 1196.18, 6.45741 }, Point { 1967.88, 1173.84 }),
      make_tuple(Frenet { 1220.78, 6.25445 }, Point { 1989.6, 1187.28 }),
      make_tuple(Frenet { 1245.93, 6.12983 }, Point { 2009.71, 1204.09 }),
      make_tuple(Frenet { 1272.1, 6.05328 }, Point { 2026.84, 1223.87 }),
      make_tuple(Frenet { 1296.67, 6.31187 }, Point { 2040.43, 1245.57 }),
      make_tuple(Frenet { 1321.17, 6.34955 }, Point { 2052.03, 1267.78 }),
      make_tuple(Frenet { 1345.73, 6.20398 }, Point { 2062.15, 1290.72 }),
      make_tuple(Frenet { 1370.9, 6.0225 }, Point { 2071.02, 1314.34 }),
      make_tuple(Frenet { 1372.4, 6.0345 }, Point { 2071.56, 1315.74 }),
      make_tuple(Frenet { 1420.77, 6.10644 }, Point { 2087.61, 1361.72 }),
      make_tuple(Frenet { 1444.84, 6.05604 }, Point { 2094.48, 1384.89 }),
      make_tuple(Frenet { 1469.34, 6.12255 }, Point { 2101.32, 1408.51 }),
      make_tuple(Frenet { 1494.69, 6.0804 }, Point { 2108, 1433.69 }),
      make_tuple(Frenet { 1519.32, 6.12662 }, Point { 2111.78, 1458.03 }),
      make_tuple(Frenet { 1543.92, 5.94998 }, Point { 2115.03, 1482.54 }),
      make_tuple(Frenet { 1568.57, 5.8933 }, Point { 2118.53, 1506.7 }),
      make_tuple(Frenet { 1593.54, 5.94355 }, Point { 2122.84, 1531.29 }),
      make_tuple(Frenet { 1618.17, 5.96469 }, Point { 2127.34, 1555.4 }),
      make_tuple(Frenet { 1642.29, 6.0106 }, Point { 2131.9, 1579.09 }),
      make_tuple(Frenet { 1667.52, 5.85315 }, Point { 2136.32, 1603.97 }),
      make_tuple(Frenet { 1691.33, 5.82439 }, Point { 2140.58, 1627.4 }),
      make_tuple(Frenet { 1716.01, 5.83623 }, Point { 2145.89, 1651.17 }),
      make_tuple(Frenet { 1740.46, 5.9794 }, Point { 2151.7, 1674.93 }),
      make_tuple(Frenet { 1765.55, 6.18472 }, Point { 2157.73, 1699.29 }),
      make_tuple(Frenet { 1789.86, 6.03089 }, Point { 2163.22, 1722.96 }),
      make_tuple(Frenet { 1814.42, 6.27566 }, Point { 2167.71, 1747.53 }),
      make_tuple(Frenet { 1838.84, 6.04622 }, Point { 2171.43, 1771.79 }),
      make_tuple(Frenet { 1863.45, 6.02855 }, Point { 2174.97, 1796.15 }),
      make_tuple(Frenet { 1887.87, 6.07527 }, Point { 2178.53, 1820.32 }),
      make_tuple(Frenet { 1912.59, 6.12885 }, Point { 2182.12, 1844.77 }),
      make_tuple(Frenet { 1937.69, 6.05513 }, Point { 2184.68, 1870.08 }),
      make_tuple(Frenet { 1962.03, 5.8779 }, Point { 2186.87, 1894.05 }),
      make_tuple(Frenet { 1985.9, 5.963 }, Point { 2190.12, 1917.43 }),
      make_tuple(Frenet { 2010.18, 5.89011 }, Point { 2194.31, 1941.34 }),
      make_tuple(Frenet { 2035.13, 6.00855 }, Point { 2199.37, 1965.55 }),
      make_tuple(Frenet { 2059.68, 6.07121 }, Point { 2204.31, 1989.78 }),
      make_tuple(Frenet { 2083.67, 5.99901 }, Point { 2208.59, 2013.38 }),
      make_tuple(Frenet { 2108.05, 5.90031 }, Point { 2212.97, 2037.35 }),
      make_tuple(Frenet { 2133.09, 5.76884 }, Point { 2218.24, 2061.56 }),
      make_tuple(Frenet { 2157.15, 5.85653 }, Point { 2224.15, 2084.47 }),
      make_tuple(Frenet { 2181.54, 6.00463 }, Point { 2231.43, 2107.77 }),
      make_tuple(Frenet { 2206.63, 6.09886 }, Point { 2238.79, 2131.76 }),
      make_tuple(Frenet { 2230.58, 6.1526 }, Point { 2245, 2155.2 }),
      make_tuple(Frenet { 2255.05, 6.06746 }, Point { 2250.49, 2179.26 }),
      make_tuple(Frenet { 2279.39, 6.06703 }, Point { 2255.49, 2203.1 }),
      make_tuple(Frenet { 2304.08, 6.06816 }, Point { 2260.34, 2227.59 }),
      make_tuple(Frenet { 2328.32, 5.9912 }, Point { 2264.13, 2251.53 }),
      make_tuple(Frenet { 2352.77, 5.95019 }, Point { 2268.83, 2275.29 }),
      make_tuple(Frenet { 2377.38, 5.70283 }, Point { 2273.56, 2299.38 }),
      make_tuple(Frenet { 2401.62, 5.72517 }, Point { 2279.87, 2321.99 }),
      make_tuple(Frenet { 2426.59, 6.01415 }, Point { 2288.36, 2345.57 }),
      make_tuple(Frenet { 2450.56, 6.0161 }, Point { 2295.86, 2368.34 }),
      make_tuple(Frenet { 2474.8, 6.18204 }, Point { 2303.58, 2391.32 }),
      make_tuple(Frenet { 2499.49, 6.19106 }, Point { 2310.96, 2415.47 }),
      make_tuple(Frenet { 2524.23, 6.07672 }, Point { 2316.36, 2439.61 }),
      make_tuple(Frenet { 2548.59, 6.04607 }, Point { 2321.16, 2463.7 }),
      make_tuple(Frenet { 2572.94, 6.02148 }, Point { 2325.53, 2487.73 }),
      make_tuple(Frenet { 2597.16, 6.05432 }, Point { 2329.84, 2511.57 }),
      make_tuple(Frenet { 2621.85, 6.12741 }, Point { 2334.29, 2535.86 }),
      make_tuple(Frenet { 2646.26, 6.21899 }, Point { 2337.06, 2560.61 }),
      make_tuple(Frenet { 2670.75, 6.02912 }, Point { 2338.1, 2585.52 }),
      make_tuple(Frenet { 2695.36, 5.98708 }, Point { 2337.8, 2610.33 }),
      make_tuple(Frenet { 2719.74, 6.00627 }, Point { 2338.05, 2634.56 }),
      make_tuple(Frenet { 2743.38, 5.98779 }, Point { 2338.09, 2658.26 }),
      make_tuple(Frenet { 2767.94, 6.00294 }, Point { 2338.89, 2682.62 }),
      make_tuple(Frenet { 2792.2, 6.00009 }, Point { 2339.08, 2707.02 }),
      make_tuple(Frenet { 2817.07, 6.01468 }, Point { 2339.3, 2731.97 }),
      make_tuple(Frenet { 2841.07, 6.06797 }, Point { 2339.32, 2755.89 }),
      make_tuple(Frenet { 2865.9, 6.32667 }, Point { 2338.79, 2781.45 }),
      make_tuple(Frenet { 2890.81, 6.30228 }, Point { 2334.83, 2806.8 }),
      make_tuple(Frenet { 2915.48, 6.25977 }, Point { 2328.05, 2831.07 }),
      make_tuple(Frenet { 2940.09, 6.24814 }, Point { 2319.27, 2854.71 }),
      make_tuple(Frenet { 2964.09, 6.00953 }, Point { 2308.74, 2877.02 }),
      make_tuple(Frenet { 2989.46, 6.25055 }, Point { 2295.89, 2898.89 }),
      make_tuple(Frenet { 3012.95, 6.46044 }, Point { 2282.74, 2918.91 }),
      make_tuple(Frenet { 3038.26, 6.41656 }, Point { 2266.55, 2939.64 }),
      make_tuple(Frenet { 3062.21, 6.10998 }, Point { 2248.93, 2956.65 }),
      make_tuple(Frenet { 3087.56, 6.4493 }, Point { 2229.78, 2973.27 }),
      make_tuple(Frenet { 3111.48, 7.02325 }, Point { 2208.14, 2986.4 }),
      make_tuple(Frenet { 3134.87, 6.63355 }, Point { 2185.05, 2995.71 }),
      make_tuple(Frenet { 3160.3, 6.25845 }, Point { 2160.18, 3001.09 }),
      make_tuple(Frenet { 3184.35, 6.39651 }, Point { 2135.39, 3003.82 }),
      make_tuple(Frenet { 3208.51, 6.14842 }, Point { 2110.82, 3004.75 }),
      make_tuple(Frenet { 3233.18, 5.98169 }, Point { 2086.18, 3004.85 }),
      make_tuple(Frenet { 3256.84, 5.99711 }, Point { 2062.58, 3005.22 }),
      make_tuple(Frenet { 3281.7, 6.137 }, Point { 2037.73, 3005.94 }),
      make_tuple(Frenet { 3305.36, 6.21366 }, Point { 2013.64, 3005.23 }),
      make_tuple(Frenet { 3329.54, 6.04988 }, Point { 1989.32, 3003.61 }),
      make_tuple(Frenet { 3353.55, 5.96716 }, Point { 1965.4, 3001.56 }),
      make_tuple(Frenet { 3377.72, 6.06404 }, Point { 1941.48, 3000.52 }),
      make_tuple(Frenet { 3401.84, 6.18806 }, Point { 1917.11, 2999 }),
      make_tuple(Frenet { 3426.65, 6.10507 }, Point { 1892.15, 2996.31 }),
      make_tuple(Frenet { 3450.6, 6.08838 }, Point { 1868.44, 2992.85 }),
      make_tuple(Frenet { 3474.82, 5.90146 }, Point { 1844.39, 2988.31 }),
      make_tuple(Frenet { 3499.02, 5.79616 }, Point { 1820.94, 2984.37 }),
      make_tuple(Frenet { 3522.89, 5.91252 }, Point { 1797.22, 2981.72 }),
      make_tuple(Frenet { 3524.39, 5.93749 }, Point { 1795.73, 2981.57 }),
      make_tuple(Frenet { 3548.01, 6.00804 }, Point { 1772.39, 2979.57 }),
      make_tuple(Frenet { 3595.78, 6.13947 }, Point { 1724.65, 2974.68 }),
      make_tuple(Frenet { 3597.29, 6.12306 }, Point { 1723.15, 2974.48 }),
      make_tuple(Frenet { 3643.98, 5.99269 }, Point { 1676.86, 2967.1 }),
      make_tuple(Frenet { 3668, 5.86445 }, Point { 1653.14, 2963.3 }),
      make_tuple(Frenet { 3692.07, 5.91212 }, Point { 1629.52, 2960.32 }),
      make_tuple(Frenet { 3716.2, 6.05321 }, Point { 1605.52, 2957.83 }),
      make_tuple(Frenet { 3740.47, 5.9231 }, Point { 1581.31, 2954.62 }),
      make_tuple(Frenet { 3765.03, 5.59048 }, Point { 1557.02, 2951.02 }),
      make_tuple(Frenet { 3789.01, 5.79639 }, Point { 1533.78, 2948.54 }),
      make_tuple(Frenet { 3812.68, 5.54879 }, Point { 1510.13, 2947.59 }),
      make_tuple(Frenet { 3836.66, 5.9395 }, Point { 1486.15, 2947.27 }),
      make_tuple(Frenet { 3860.84, 5.87304 }, Point { 1462.09, 2946.84 }),
      make_tuple(Frenet { 3884.78, 5.9107 }, Point { 1438.14, 2946.6 }),
      make_tuple(Frenet { 3908.93, 5.85463 }, Point { 1414.16, 2946.67 }),
      make_tuple(Frenet { 3932.79, 5.93396 }, Point { 1390.3, 2947.11 }),
      make_tuple(Frenet { 3957.07, 5.73686 }, Point { 1366.19, 2947.79 }),
      make_tuple(Frenet { 3980.96, 5.59945 }, Point { 1342.31, 2948.66 }),
      make_tuple(Frenet { 4005.27, 5.80873 }, Point { 1318.58, 2950.31 }),
      make_tuple(Frenet { 4029.41, 5.96486 }, Point { 1294.66, 2953.57 }),
      make_tuple(Frenet { 4052.7, 6.07328 }, Point { 1271.2, 2956.57 }),
      make_tuple(Frenet { 4076.89, 6.42459 }, Point { 1247.1, 2958.63 }),
      make_tuple(Frenet { 4101.03, 6.0527 }, Point { 1222.56, 2959.86 }),
      make_tuple(Frenet { 4124.67, 6.39523 }, Point { 1198.92, 2960.19 }),
      make_tuple(Frenet { 4148.86, 6.20097 }, Point { 1174.73, 2959.98 }),
      make_tuple(Frenet { 4173.62, 6.30541 }, Point { 1149.66, 2959.34 }),
      make_tuple(Frenet { 4196.72, 6.0093 }, Point { 1126.2, 2957.76 }),
      make_tuple(Frenet { 4221.27, 6.59302 }, Point { 1101.76, 2955.34 }),
      make_tuple(Frenet { 4222.78, 6.60724 }, Point { 1100.26, 2955.17 }),
      make_tuple(Frenet { 4269.41, 6.61093 }, Point { 1053.67, 2946.9 }),
      make_tuple(Frenet { 4293.14, 6.48509 }, Point { 1030.64, 2941.2 }),
      make_tuple(Frenet { 4295.15, 6.44975 }, Point { 1028.7, 2940.7 }),
      make_tuple(Frenet { 4341.03, 5.94862 }, Point { 984.274, 2927.99 }),
      make_tuple(Frenet { 4365.11, 5.81777 }, Point { 961.248, 2920.94 }),
      make_tuple(Frenet { 4388.63, 5.4645 }, Point { 938.937, 2915.8 }),
      make_tuple(Frenet { 4412.86, 5.84677 }, Point { 915.52, 2912.17 }),
      make_tuple(Frenet { 4436.49, 5.61519 }, Point { 892.02, 2909.69 }),
      make_tuple(Frenet { 4460.2, 5.92491 }, Point { 868.667, 2907.94 }),
      make_tuple(Frenet { 4484.1, 5.5453 }, Point { 844.818, 2906.47 }),
      make_tuple(Frenet { 4507.72, 5.64997 }, Point { 821.215, 2905.49 }),
      make_tuple(Frenet { 4532.26, 5.66299 }, Point { 797.216, 2905.53 }),
      make_tuple(Frenet { 4555.43, 5.92335 }, Point { 774.076, 2906.83 }),
      make_tuple(Frenet { 4579.26, 6.11583 }, Point { 750.187, 2907.95 }),
      make_tuple(Frenet { 4603.27, 6.30816 }, Point { 726.191, 2908.88 }),
      make_tuple(Frenet { 4604.79, 6.3017 }, Point { 724.678, 2908.92 }),
      make_tuple(Frenet { 4650.92, 6.31792 }, Point { 678.13, 2908.16 }),
      make_tuple(Frenet { 4674.9, 6.03846 }, Point { 654.177, 2906.94 }),
      make_tuple(Frenet { 4676.4, 6.01854 }, Point { 652.681, 2906.86 }),
      make_tuple(Frenet { 4723.28, 6.28617 }, Point { 605.172, 2902.89 }),
      make_tuple(Frenet { 4724.79, 6.32355 }, Point { 603.673, 2902.66 }),
      make_tuple(Frenet { 4770.72, 6.10116 }, Point { 558.144, 2894.16 }),
      make_tuple(Frenet { 4794.75, 6.45622 }, Point { 534.717, 2888.82 }),
      make_tuple(Frenet { 4818.52, 6.01738 }, Point { 511.725, 2882.76 }),
      make_tuple(Frenet { 4842.19, 6.40458 }, Point { 488.668, 2875.5 }),
      make_tuple(Frenet { 4843.7, 6.39239 }, Point { 487.235, 2875 }),
      make_tuple(Frenet { 4890.01, 6.73743 }, Point { 443.384, 2859.09 }),
      make_tuple(Frenet { 4914.11, 6.38042 }, Point { 421.112, 2849.9 }),
      make_tuple(Frenet { 4938.41, 7.06554 }, Point { 398.72, 2837.68 }),
      make_tuple(Frenet { 4961.46, 6.013 }, Point { 379.186, 2823.78 }),
      make_tuple(Frenet { 4985.54, 7.00166 }, Point { 360.657, 2808.22 }),
      make_tuple(Frenet { 5010.28, 6.16795 }, Point { 342.938, 2790.93 }),
      make_tuple(Frenet { 5033.27, 7.06033 }, Point { 326.98, 2773.07 }),
      make_tuple(Frenet { 5057.81, 6.91327 }, Point { 311.724, 2753.85 }),
      make_tuple(Frenet { 5080.62, 6.60567 }, Point { 298.353, 2734.03 }),
      make_tuple(Frenet { 5104.8, 6.90576 }, Point { 286.46, 2712.97 }),
      make_tuple(Frenet { 5129.42, 6.19781 }, Point { 275.243, 2691.04 }),
      make_tuple(Frenet { 5152.24, 6.48725 }, Point { 264.876, 2670.24 }),
      make_tuple(Frenet { 5176.89, 6.46929 }, Point { 254.579, 2647.84 }),
      make_tuple(Frenet { 5178.91, 6.42078 }, Point { 253.777, 2645.99 }),
      make_tuple(Frenet { 5223.76, 6.22496 }, Point { 238.113, 2603.32 }),
      make_tuple(Frenet { 5247.39, 5.99498 }, Point { 230.641, 2580.93 }),
      make_tuple(Frenet { 5271.06, 6.06026 }, Point { 222.775, 2558.6 }),
      make_tuple(Frenet { 5294.91, 6.01553 }, Point { 214.954, 2536.07 }),
      make_tuple(Frenet { 5318.48, 6.21337 }, Point { 207.416, 2513.6 }),
      make_tuple(Frenet { 5319.99, 6.2224 }, Point { 206.942, 2512.17 }),
      make_tuple(Frenet { 5366.15, 6.23357 }, Point { 193.251, 2467.78 }),
      make_tuple(Frenet { 5390, 6.2281 }, Point { 186.982, 2444.78 }),
      make_tuple(Frenet { 5413.39, 6.11244 }, Point { 181.077, 2421.91 }),
      make_tuple(Frenet { 5414.89, 6.14245 }, Point { 180.707, 2420.45 }),
      make_tuple(Frenet { 5461.15, 6.28784 }, Point { 170.806, 2374.78 }),
      make_tuple(Frenet { 5485.11, 6.30535 }, Point { 167.219, 2351.1 }),
      make_tuple(Frenet { 5508.48, 6.20671 }, Point { 164.292, 2327.57 }),
      make_tuple(Frenet { 5532.9, 6.22081 }, Point { 162.016, 2303.25 }),
      make_tuple(Frenet { 5556.42, 6.14956 }, Point { 160.486, 2279.49 }),
      make_tuple(Frenet { 5580.16, 6.02991 }, Point { 159.55, 2255.77 }),
      make_tuple(Frenet { 5581.65, 6.02035 }, Point { 159.493, 2254.27 }),
      make_tuple(Frenet { 5626.86, 6.11753 }, Point { 157.119, 2209.18 }),
      make_tuple(Frenet { 5650.33, 6.13988 }, Point { 156.194, 2185.51 }),
      make_tuple(Frenet { 5674.6, 6.22919 }, Point { 155.704, 2161.25 }),
      make_tuple(Frenet { 5697.92, 6.13242 }, Point { 155.681, 2137.68 }),
      make_tuple(Frenet { 5721.6, 6.53985 }, Point { 155.846, 2113.99 }),
      make_tuple(Frenet { 5745.23, 6.24012 }, Point { 156.717, 2090.38 }),
      make_tuple(Frenet { 5768.61, 6.53257 }, Point { 159.426, 2066.36 }),
      make_tuple(Frenet { 5770.14, 6.51908 }, Point { 159.673, 2064.86 }),
      make_tuple(Frenet { 5816.04, 6.33924 }, Point { 167.8, 2019.44 }),
      make_tuple(Frenet { 5840, 6.14348 }, Point { 172.551, 1995.95 }),
      make_tuple(Frenet { 5863.15, 6.65648 }, Point { 177.955, 1972.96 }),
      make_tuple(Frenet { 5888, 6.49747 }, Point { 184.665, 1949.04 }),
      make_tuple(Frenet { 5910.5, 6.64471 }, Point { 193.125, 1927.12 }),
      make_tuple(Frenet { 5934.44, 6.33563 }, Point { 203.365, 1905.47 }),
      make_tuple(Frenet { 5957.82, 6.04745 }, Point { 213.516, 1884.29 }),
      make_tuple(Frenet { 5981.41, 5.99699 }, Point { 223.784, 1863.04 }),
      make_tuple(Frenet { 6005.02, 5.93773 }, Point { 233.958, 1841.8 }),
      make_tuple(Frenet { 6028.54, 5.7862 }, Point { 244.093, 1820.58 }),
      make_tuple(Frenet { 6052.54, 5.99773 }, Point { 254.107, 1798.77 }),
      make_tuple(Frenet { 6075.7, 5.73494 }, Point { 263.111, 1777.72 }),
      make_tuple(Frenet { 6099.08, 5.85835 }, Point { 271.945, 1756.08 }),
      make_tuple(Frenet { 6122.66, 5.91975 }, Point { 280.58, 1734.3 }),
      make_tuple(Frenet { 6146.09, 6.00218 }, Point { 288.909, 1712.4 }),
      make_tuple(Frenet { 6169.47, 6.01399 }, Point { 297.353, 1690.47 }),
      make_tuple(Frenet { 6193.02, 6.25506 }, Point { 306.009, 1668.57 }),
      make_tuple(Frenet { 6216.23, 6.12027 }, Point { 315.3, 1646.87 }),
      make_tuple(Frenet { 6240.48, 6.2375 }, Point { 325.808, 1625.01 }),
      make_tuple(Frenet { 6263.28, 6.085 }, Point { 336.23, 1604.5 }),
      make_tuple(Frenet { 6286.85, 6.11815 }, Point { 347.216, 1583.65 }),
      make_tuple(Frenet { 6310.71, 6.013 }, Point { 358.51, 1562.56 }),
      make_tuple(Frenet { 6333.67, 6.16876 }, Point { 369.322, 1542.3 }),
      make_tuple(Frenet { 6356.94, 6.00625 }, Point { 380.688, 1521.73 }),
      make_tuple(Frenet { 6381.03, 6.11645 }, Point { 392.912, 1500.97 }),
      make_tuple(Frenet { 6403.98, 5.99143 }, Point { 404.752, 1481.31 }),
      make_tuple(Frenet { 6427.47, 5.92034 }, Point { 416.661, 1461.15 }),
      make_tuple(Frenet { 6451.31, 5.86885 }, Point { 428.646, 1440.54 }),
      make_tuple(Frenet { 6452.8, 5.8753 }, Point { 429.388, 1439.24 }),
      make_tuple(Frenet { 6498.08, 6.09036 }, Point { 450.927, 1399.62 }),
      make_tuple(Frenet { 6520.8, 6.01666 }, Point { 461.923, 1379.46 }),
      make_tuple(Frenet { 6544.28, 6.2999 }, Point { 473.655, 1359.12 }),
      make_tuple(Frenet { 6567.89, 6.18304 }, Point { 485.8, 1338.88 }),
      make_tuple(Frenet { 6569.39, 6.15999 }, Point { 486.589, 1337.59 }),
      make_tuple(Frenet { 6614.59, 6.38711 }, Point { 511.099, 1299.28 }),
      make_tuple(Frenet { 6637.49, 6.13098 }, Point { 524.359, 1280.09 }),
      make_tuple(Frenet { 6638.99, 6.18459 }, Point { 525.228, 1278.86 }),
      make_tuple(Frenet { 6685.8, 6.09148 }, Point { 553.727, 1241.73 }),
      make_tuple(Frenet { 6708.23, 6.74706 }, Point { 570.038, 1224.97 }),
      make_tuple(Frenet { 6732.06, 6.13687 }, Point { 587.72, 1208.98 }),
      make_tuple(Frenet { 6755, 6.6585 }, Point { 605.34, 1193.56 }),
      make_tuple(Frenet { 6779.29, 6.26212 }, Point { 624.412, 1178.51 }),
      make_tuple(Frenet { 6802.12, 6.89809 }, Point { 644.574, 1165.71 }),
      make_tuple(Frenet { 6826.85, 6.10113 }, Point { 666.525, 1154.29 }),
      make_tuple(Frenet { 6828.36, 5.99628 }, Point { 667.898, 1153.64 }),
      make_tuple(Frenet { 6872.69, 6.13434 }, Point { 711.307, 1137.26 }),
      make_tuple(Frenet { 6896.83, 6.77742 }, Point { 734.998, 1132.61 }),
      make_tuple(Frenet { 6920.85, 6.32469 }, Point { 759.734, 1129.91 }), };

  for (const tuple<Frenet, Point>& frenetPointTuple : frenetPointTuples) {
    const Frenet& frenet = get<0>(frenetPointTuple);
    const Point& point = get<1>(frenetPointTuple);
    test_convert(point, frenet);
  }
}
