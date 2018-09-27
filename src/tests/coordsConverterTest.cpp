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
    FrenetCart(Frenet {125.22, 6.04427}, Point {909.866, 1128.79}),
    FrenetCart(Frenet {150.506, 6.00188}, Point {935.491, 1129.08}),
    FrenetCart(Frenet {175.962, 6.22199}, Point {961.079, 1131.65}),
    FrenetCart(Frenet {178.011, 6.13349}, Point {963.107, 1131.96}),
    FrenetCart(Frenet {201.93, 6.522}, Point {987.256, 1136.79}),
    FrenetCart(Frenet {251.524, 5.98363}, Point {1034.33, 1155.14}),
    FrenetCart(Frenet {276.481, 6.00181}, Point {1057.48, 1164.59}),
    FrenetCart(Frenet {300.757, 5.89808}, Point {1079.84, 1174.05}),
    FrenetCart(Frenet {325.307, 5.96525}, Point {1102.74, 1179.88}),
    FrenetCart(Frenet {327.442, 5.93641}, Point {1104.17, 1180.15}),
    FrenetCart(Frenet {374.839, 5.89818}, Point {1150.98, 1185.01}),
    FrenetCart(Frenet {399.717, 5.97431}, Point {1175.65, 1186.37}),
    FrenetCart(Frenet {424.639, 5.97421}, Point {1200.54, 1187.53}),
    FrenetCart(Frenet {426.631, 5.97634}, Point {1202.53, 1187.62}),
    FrenetCart(Frenet {450.543, 5.83569}, Point {1226.35, 1188.64}),
    FrenetCart(Frenet {498.904, 5.97723}, Point {1274.21, 1187.54}),
    FrenetCart(Frenet {523.603, 5.83817}, Point {1298.89, 1186.57}),
    FrenetCart(Frenet {548.384, 5.89684}, Point {1323.17, 1184.31}),
    FrenetCart(Frenet {573.303, 6.03968}, Point {1348.06, 1181.55}),
    FrenetCart(Frenet {598.182, 5.98261}, Point {1372.8, 1179.31}),
    FrenetCart(Frenet {622.795, 5.993}, Point {1397.3, 1176.92}),
    FrenetCart(Frenet {624.477, 5.98233}, Point {1398.77, 1176.75}),
    FrenetCart(Frenet {672.578, 6.00997}, Point {1446.5, 1170.55}),
    FrenetCart(Frenet {698.349, 6.0105}, Point {1472.06, 1167.33}),
    FrenetCart(Frenet {722.229, 6.04014}, Point {1495.91, 1164.36}),
    FrenetCart(Frenet {747.132, 6.01715}, Point {1520.84, 1161.88}),
    FrenetCart(Frenet {771.928, 5.99479}, Point {1545.56, 1160.01}),
    FrenetCart(Frenet {796.882, 6.01815}, Point {1570.32, 1157.74}),
    FrenetCart(Frenet {821.543, 5.97467}, Point {1595.11, 1156.19}),
    FrenetCart(Frenet {846.265, 5.92733}, Point {1619.59, 1154.12}),
    FrenetCart(Frenet {871.138, 6.08612}, Point {1644.05, 1151.09}),
    FrenetCart(Frenet {895.994, 6.00791}, Point {1669.19, 1149.2}),
    FrenetCart(Frenet {920.664, 5.98983}, Point {1693.66, 1147.28}),
    FrenetCart(Frenet {945.435, 6.05136}, Point {1718.41, 1145.21}),
    FrenetCart(Frenet {970.329, 6.03073}, Point {1743.36, 1143.54}),
    FrenetCart(Frenet {994.989, 5.99843}, Point {1767.93, 1142.13}),
    FrenetCart(Frenet {1019.75, 6.02273}, Point {1792.9, 1140.53}),
    FrenetCart(Frenet {1045.13, 6.12257}, Point {1818.28, 1139.9}),
    FrenetCart(Frenet {1069.87, 6.11907}, Point {1843.63, 1142.07}),
    FrenetCart(Frenet {1095.01, 6.22986}, Point {1868.8, 1145.1}),
    FrenetCart(Frenet {1119.24, 6.23904}, Point {1893.15, 1149.54}),
    FrenetCart(Frenet {1144.06, 6.20454}, Point {1917.53, 1155.5}),
    FrenetCart(Frenet {1169.03, 6.34618}, Point {1941.97, 1162.86}),
    FrenetCart(Frenet {1194.27, 6.37969}, Point {1966.18, 1172.95}),
    FrenetCart(Frenet {1219.35, 6.16312}, Point {1988.42, 1186.46}),
    FrenetCart(Frenet {1244.71, 6.01115}, Point {2008.65, 1203.06}),
    FrenetCart(Frenet {1270.57, 6.17753}, Point {2025.92, 1222.63}),
    FrenetCart(Frenet {1295.13, 6.36256}, Point {2039.68, 1244.23}),
    FrenetCart(Frenet {1319.61, 6.35132}, Point {2051.35, 1266.38}),
    FrenetCart(Frenet {1321.13, 6.34969}, Point {2052.02, 1267.75}),
    FrenetCart(Frenet {1369.35, 6.01189}, Point {2070.47, 1312.89}),
    FrenetCart(Frenet {1394.15, 6.12971}, Point {2079.27, 1336.08}),
    FrenetCart(Frenet {1418.85, 6.12508}, Point {2087.07, 1359.88}),
    FrenetCart(Frenet {1443.42, 6.05493}, Point {2094.08, 1383.53}),
    FrenetCart(Frenet {1468.43, 6.10449}, Point {2101.05, 1407.64}),
    FrenetCart(Frenet {1493.39, 6.00381}, Point {2107.7, 1432.2}),
    FrenetCart(Frenet {1518.35, 6.14555}, Point {2111.65, 1457.07}),
    FrenetCart(Frenet {1519.85, 6.11694}, Point {2111.85, 1458.56}),
    FrenetCart(Frenet {1566.89, 5.90759}, Point {2118.26, 1505.04}),
    FrenetCart(Frenet {1591.36, 5.92643}, Point {2122.45, 1529.14}),
    FrenetCart(Frenet {1616.98, 5.96191}, Point {2127.12, 1554.24}),
    FrenetCart(Frenet {1640.6, 6.01243}, Point {2131.58, 1577.42}),
    FrenetCart(Frenet {1665.33, 5.87758}, Point {2135.95, 1601.81}),
    FrenetCart(Frenet {1689.69, 5.79885}, Point {2140.26, 1625.79}),
    FrenetCart(Frenet {1714.36, 5.8386}, Point {2145.51, 1649.57}),
    FrenetCart(Frenet {1715.35, 5.83735}, Point {2145.74, 1650.54}),
    FrenetCart(Frenet {1763.38, 6.1754}, Point {2157.22, 1697.18}),
    FrenetCart(Frenet {1788.17, 6.06108}, Point {2162.86, 1721.31}),
    FrenetCart(Frenet {1812.72, 6.29433}, Point {2167.45, 1745.85}),
    FrenetCart(Frenet {1837.14, 6.0354}, Point {2171.17, 1770.11}),
    FrenetCart(Frenet {1861.76, 6.03378}, Point {2174.73, 1794.47}),
    FrenetCart(Frenet {1887.18, 6.06915}, Point {2178.43, 1819.63}),
    FrenetCart(Frenet {1910.86, 6.14525}, Point {2181.89, 1843.06}),
    FrenetCart(Frenet {1935.97, 6.07687}, Point {2184.55, 1868.36}),
    FrenetCart(Frenet {1959.86, 5.91188}, Point {2186.62, 1891.89}),
    FrenetCart(Frenet {1984.23, 5.99438}, Point {2189.86, 1915.78}),
    FrenetCart(Frenet {2008.52, 5.87517}, Point {2194, 1939.71}),
    FrenetCart(Frenet {2032.96, 5.98845}, Point {2198.89, 1963.43}),
    FrenetCart(Frenet {2034.46, 6.002}, Point {2199.22, 1964.89}),
    FrenetCart(Frenet {2082.16, 5.99931}, Point {2208.32, 2011.91}),
    FrenetCart(Frenet {2107.08, 5.89315}, Point {2212.78, 2036.41}),
    FrenetCart(Frenet {2109.55, 5.91578}, Point {2213.26, 2038.83}),
    FrenetCart(Frenet {2155.35, 5.87808}, Point {2223.65, 2082.75}),
    FrenetCart(Frenet {2179.75, 6.00081}, Point {2230.91, 2106.04}),
    FrenetCart(Frenet {2204.76, 6.11207}, Point {2238.26, 2129.97}),
    FrenetCart(Frenet {2228.71, 6.15981}, Point {2244.56, 2153.38}),
    FrenetCart(Frenet {2253.16, 6.0741}, Point {2250.11, 2177.42}),
    FrenetCart(Frenet {2277.52, 6.05305}, Point {2255.09, 2201.27}),
    FrenetCart(Frenet {2302.14, 6.05549}, Point {2260.02, 2225.68}),
    FrenetCart(Frenet {2326.5, 5.96565}, Point {2263.82, 2249.74}),
    FrenetCart(Frenet {2350.91, 5.93784}, Point {2268.45, 2273.48}),
    FrenetCart(Frenet {2375.59, 5.72564}, Point {2273.22, 2297.62}),
    FrenetCart(Frenet {2399.88, 5.72448}, Point {2279.29, 2320.34}),
    FrenetCart(Frenet {2424.36, 6.0038}, Point {2287.61, 2343.37}),
    FrenetCart(Frenet {2448.76, 6.02157}, Point {2295.3, 2366.63}),
    FrenetCart(Frenet {2450.76, 6.01529}, Point {2295.92, 2368.53}),
    FrenetCart(Frenet {2498.12, 6.1547}, Point {2310.62, 2414.14}),
    FrenetCart(Frenet {2522.39, 6.10891}, Point {2315.98, 2437.8}),
    FrenetCart(Frenet {2546.74, 6.05956}, Point {2320.82, 2461.88}),
    FrenetCart(Frenet {2571.08, 6.02441}, Point {2325.2, 2485.91}),
    FrenetCart(Frenet {2595.82, 6.03918}, Point {2329.59, 2510.25}),
    FrenetCart(Frenet {2598.31, 6.06753}, Point {2330.06, 2512.7}),
    FrenetCart(Frenet {2644.8, 6.23968}, Point {2336.95, 2559.16}),
    FrenetCart(Frenet {2669.31, 6.05825}, Point {2338.1, 2584.08}),
    FrenetCart(Frenet {2693.48, 5.97488}, Point {2337.81, 2608.44}),
    FrenetCart(Frenet {2694.95, 5.984}, Point {2337.8, 2609.92}),
    FrenetCart(Frenet {2742.08, 5.97095}, Point {2338.07, 2656.97}),
    FrenetCart(Frenet {2766.57, 6.01785}, Point {2338.86, 2681.24}),
    FrenetCart(Frenet {2790.86, 6.00124}, Point {2339.07, 2705.68}),
    FrenetCart(Frenet {2815.7, 6.012}, Point {2339.3, 2730.61}),
    FrenetCart(Frenet {2839.25, 6.03414}, Point {2339.28, 2754.07}),
    FrenetCart(Frenet {2863.97, 6.27377}, Point {2338.96, 2779.52}),
    FrenetCart(Frenet {2865.51, 6.31688}, Point {2338.82, 2781.06}),
    FrenetCart(Frenet {2914.49, 6.23913}, Point {2328.35, 2830.13}),
    FrenetCart(Frenet {2939.09, 6.2168}, Point {2319.66, 2853.78}),
    FrenetCart(Frenet {2964.24, 5.99997}, Point {2308.98, 2876.56}),
    FrenetCart(Frenet {2988.43, 6.27421}, Point {2296.45, 2898.02}),
    FrenetCart(Frenet {3012.4, 6.45238}, Point {2283.05, 2918.45}),
    FrenetCart(Frenet {3037.14, 6.37581}, Point {2267.31, 2938.81}),
    FrenetCart(Frenet {3061.65, 6.08428}, Point {2249.35, 2956.26}),
    FrenetCart(Frenet {3063.14, 6.15254}, Point {2248.25, 2957.28}),
    FrenetCart(Frenet {3111.42, 7.02354}, Point {2208.19, 2986.37}),
    FrenetCart(Frenet {3134.81, 6.62902}, Point {2185.11, 2995.69}),
    FrenetCart(Frenet {3160.24, 6.26294}, Point {2160.25, 3001.08}),
    FrenetCart(Frenet {3184.8, 6.38714}, Point {2134.95, 3003.84}),
    FrenetCart(Frenet {3208.45, 6.14902}, Point {2110.89, 3004.75}),
    FrenetCart(Frenet {3232.62, 5.98317}, Point {2086.75, 3004.84}),
    FrenetCart(Frenet {3256.78, 5.99763}, Point {2062.64, 3005.21}),
    FrenetCart(Frenet {3281.12, 6.14462}, Point {2038.31, 3005.94}),
    FrenetCart(Frenet {3305.8, 6.21135}, Point {2013.2, 3005.2}),
    FrenetCart(Frenet {3329.47, 6.05042}, Point {1989.39, 3003.61}),
    FrenetCart(Frenet {3330.99, 6.04605}, Point {1987.88, 3003.49}),
    FrenetCart(Frenet {3377.41, 6.06225}, Point {1941.79, 3000.54}),
    FrenetCart(Frenet {3401.53, 6.18555}, Point {1917.42, 2999.03}),
    FrenetCart(Frenet {3425.83, 6.09124}, Point {1892.96, 2996.41}),
    FrenetCart(Frenet {3450.28, 6.09382}, Point {1868.75, 2992.9}),
    FrenetCart(Frenet {3475, 5.89913}, Point {1844.21, 2988.27}),
    FrenetCart(Frenet {3498.72, 5.80001}, Point {1821.25, 2984.41}),
    FrenetCart(Frenet {3522.58, 5.90791}, Point {1797.53, 2981.75}),
    FrenetCart(Frenet {3546.71, 5.99694}, Point {1773.69, 2979.67}),
    FrenetCart(Frenet {3570.91, 6.04812}, Point {1749.35, 2977.6}),
    FrenetCart(Frenet {3595.25, 6.14538}, Point {1725.17, 2974.75}),
    FrenetCart(Frenet {3619.85, 6.09396}, Point {1700.61, 2971.08}),
    FrenetCart(Frenet {3643.45, 5.99607}, Point {1677.38, 2967.19}),
    FrenetCart(Frenet {3667.48, 5.8617}, Point {1653.65, 2963.38}),
    FrenetCart(Frenet {3691.55, 5.91025}, Point {1630.04, 2960.38}),
    FrenetCart(Frenet {3693.53, 5.91859}, Point {1628.06, 2960.17}),
    FrenetCart(Frenet {3740.08, 5.92873}, Point {1581.7, 2954.68}),
    FrenetCart(Frenet {3764.15, 5.59259}, Point {1557.89, 2951.14}),
    FrenetCart(Frenet {3788.64, 5.81153}, Point {1534.15, 2948.57}),
    FrenetCart(Frenet {3812.31, 5.54481}, Point {1510.5, 2947.6}),
    FrenetCart(Frenet {3836.28, 5.93315}, Point {1486.53, 2947.28}),
    FrenetCart(Frenet {3860.47, 5.87393}, Point {1462.46, 2946.84}),
    FrenetCart(Frenet {3884.41, 5.90843}, Point {1438.52, 2946.61}),
    FrenetCart(Frenet {3909.05, 5.85372}, Point {1414.03, 2946.67}),
    FrenetCart(Frenet {3932.42, 5.92958}, Point {1390.67, 2947.1}),
    FrenetCart(Frenet {3956.7, 5.74233}, Point {1366.56, 2947.78}),
    FrenetCart(Frenet {3980.59, 5.59663}, Point {1342.68, 2948.64}),
    FrenetCart(Frenet {4005.41, 5.80598}, Point {1318.44, 2950.32}),
    FrenetCart(Frenet {4007.36, 5.76611}, Point {1316.5, 2950.54}),
    FrenetCart(Frenet {4052.7, 6.07301}, Point {1271.21, 2956.57}),
    FrenetCart(Frenet {4076.88, 6.42537}, Point {1247.11, 2958.63}),
    FrenetCart(Frenet {4101.02, 6.05269}, Point {1222.58, 2959.86}),
    FrenetCart(Frenet {4124.66, 6.39523}, Point {1198.93, 2960.19}),
    FrenetCart(Frenet {4126.17, 6.39609}, Point {1197.43, 2960.19}),
    FrenetCart(Frenet {4172.91, 6.29789}, Point {1150.37, 2959.37}),
    FrenetCart(Frenet {4197.02, 6.01813}, Point {1125.9, 2957.73}),
    FrenetCart(Frenet {4221.07, 6.59118}, Point {1101.96, 2955.37}),
    FrenetCart(Frenet {4245.78, 6.12885}, Point {1077.49, 2951.9}),
    FrenetCart(Frenet {4269.73, 6.61429}, Point {1053.36, 2946.83}),
    FrenetCart(Frenet {4293.45, 6.4799}, Point {1030.34, 2941.12}),
    FrenetCart(Frenet {4317.05, 6.05192}, Point {1007.22, 2934.97}),
    FrenetCart(Frenet {4341.35, 5.94215}, Point {983.966, 2927.89}),
    FrenetCart(Frenet {4364.93, 5.81378}, Point {961.421, 2920.99}),
    FrenetCart(Frenet {4388.94, 5.46484}, Point {938.638, 2915.74}),
    FrenetCart(Frenet {4412.67, 5.85365}, Point {915.711, 2912.2}),
    FrenetCart(Frenet {4414.15, 5.8092}, Point {914.241, 2912.01}),
    FrenetCart(Frenet {4460.21, 5.92509}, Point {868.663, 2907.94}),
    FrenetCart(Frenet {4484.1, 5.54548}, Point {844.814, 2906.47}),
    FrenetCart(Frenet {4507.72, 5.65015}, Point {821.211, 2905.49}),
    FrenetCart(Frenet {4531.77, 5.66625}, Point {797.703, 2905.51}),
    FrenetCart(Frenet {4555.44, 5.92317}, Point {774.072, 2906.83}),
    FrenetCart(Frenet {4579.27, 6.11568}, Point {750.182, 2907.95}),
    FrenetCart(Frenet {4603.78, 6.30667}, Point {725.682, 2908.89}),
    FrenetCart(Frenet {4626.86, 6.19094}, Point {702.175, 2908.99}),
    FrenetCart(Frenet {4650.93, 6.31807}, Point {678.126, 2908.16}),
    FrenetCart(Frenet {4674.91, 6.03861}, Point {654.173, 2906.94}),
    FrenetCart(Frenet {4676.91, 6.01254}, Point {652.179, 2906.83}),
    FrenetCart(Frenet {4723.28, 6.28591}, Point {605.168, 2902.89}),
    FrenetCart(Frenet {4746.91, 6.45217}, Point {581.867, 2898.93}),
    FrenetCart(Frenet {4770.34, 6.08923}, Point {558.518, 2894.24}),
    FrenetCart(Frenet {4794.36, 6.45555}, Point {535.092, 2888.91}),
    FrenetCart(Frenet {4818.64, 6.01226}, Point {511.611, 2882.73}),
    FrenetCart(Frenet {4842.31, 6.40368}, Point {488.555, 2875.46}),
    FrenetCart(Frenet {4866.25, 6.10384}, Point {465.712, 2867.25}),
    FrenetCart(Frenet {4890.14, 6.73932}, Point {443.27, 2859.05}),
    FrenetCart(Frenet {4914.23, 6.37376}, Point {420.999, 2849.84}),
    FrenetCart(Frenet {4937.51, 7.05902}, Point {399.484, 2838.16}),
    FrenetCart(Frenet {4961.49, 6.02336}, Point {379.084, 2823.7}),
    FrenetCart(Frenet {4963.51, 6.1681}, Point {377.491, 2822.44}),
    FrenetCart(Frenet {5010.41, 6.15827}, Point {342.845, 2790.83}),
    FrenetCart(Frenet {5011.94, 6.03902}, Point {341.799, 2789.72}),
    FrenetCart(Frenet {5057.95, 6.90756}, Point {311.64, 2753.74}),
    FrenetCart(Frenet {5080.76, 6.61233}, Point {298.278, 2733.91}),
    FrenetCart(Frenet {5104.94, 6.90365}, Point {286.393, 2712.84}),
    FrenetCart(Frenet {5129.06, 6.2105}, Point {275.404, 2691.36}),
    FrenetCart(Frenet {5131.06, 6.14157}, Point {274.504, 2689.58}),
    FrenetCart(Frenet {5176.52, 6.47741}, Point {254.722, 2648.18}),
    FrenetCart(Frenet {5199.88, 6.33264}, Point {245.774, 2625.94}),
    FrenetCart(Frenet {5201.4, 6.35633}, Point {245.256, 2624.51}),
    FrenetCart(Frenet {5247.51, 5.995}, Point {230.603, 2580.82}),
    FrenetCart(Frenet {5271.18, 6.06084}, Point {222.737, 2558.5}),
    FrenetCart(Frenet {5295.03, 6.01412}, Point {214.917, 2535.96}),
    FrenetCart(Frenet {5296.9, 6.00695}, Point {214.276, 2534.06}),
    FrenetCart(Frenet {5342.49, 6.16831}, Point {200.044, 2490.75}),
    FrenetCart(Frenet {5366.49, 6.23666}, Point {193.159, 2467.46}),
    FrenetCart(Frenet {5390.34, 6.22525}, Point {186.896, 2444.45}),
    FrenetCart(Frenet {5413.72, 6.11928}, Point {180.994, 2421.59}),
    FrenetCart(Frenet {5437.14, 6.3282}, Point {175.482, 2398.82}),
    FrenetCart(Frenet {5460.52, 6.27613}, Point {170.913, 2375.41}),
    FrenetCart(Frenet {5462.03, 6.30273}, Point {170.661, 2373.92}),
    FrenetCart(Frenet {5508.17, 6.20163}, Point {164.325, 2327.87}),
    FrenetCart(Frenet {5532.09, 6.23181}, Point {162.08, 2304.05}),
    FrenetCart(Frenet {5555.61, 6.14685}, Point {160.525, 2280.3}),
    FrenetCart(Frenet {5579.35, 6.03548}, Point {159.581, 2256.57}),
    FrenetCart(Frenet {5602.89, 6.00451}, Point {158.489, 2233.1}),
    FrenetCart(Frenet {5626.55, 6.11759}, Point {157.135, 2209.48}),
    FrenetCart(Frenet {5650.02, 6.13609}, Point {156.203, 2185.82}),
    FrenetCart(Frenet {5673.78, 6.23363}, Point {155.713, 2162.06}),
    FrenetCart(Frenet {5697.11, 6.11544}, Point {155.679, 2138.49}),
    FrenetCart(Frenet {5720.78, 6.53316}, Point {155.833, 2114.81}),
    FrenetCart(Frenet {5744.9, 6.25246}, Point {156.697, 2090.7}),
    FrenetCart(Frenet {5768.28, 6.53457}, Point {159.374, 2066.68}),
    FrenetCart(Frenet {5792.12, 6.00425}, Point {163.546, 2043.22}),
    FrenetCart(Frenet {5815.49, 6.3351}, Point {167.701, 2019.98}),
    FrenetCart(Frenet {5839.45, 6.1587}, Point {172.43, 1996.49}),
    FrenetCart(Frenet {5862.59, 6.64214}, Point {177.82, 1973.51}),
    FrenetCart(Frenet {5886.9, 6.54749}, Point {184.328, 1950.08}),
    FrenetCart(Frenet {5909.91, 6.63654}, Point {192.887, 1927.66}),
    FrenetCart(Frenet {5934.87, 6.32466}, Point {203.551, 1905.09}),
    FrenetCart(Frenet {5957.23, 6.04524}, Point {213.265, 1884.82}),
    FrenetCart(Frenet {5980.84, 5.99887}, Point {223.531, 1863.56}),
    FrenetCart(Frenet {6004.44, 5.94237}, Point {233.707, 1842.32}),
    FrenetCart(Frenet {6027.96, 5.78777}, Point {243.847, 1821.1}),
    FrenetCart(Frenet {6051.2, 5.96834}, Point {253.564, 1799.99}),
    FrenetCart(Frenet {6075.85, 5.73437}, Point {263.169, 1777.58}),
    FrenetCart(Frenet {6098.24, 5.84976}, Point {271.63, 1756.86}),
    FrenetCart(Frenet {6121.82, 5.92122}, Point {280.278, 1735.08}),
    FrenetCart(Frenet {6145.25, 5.99788}, Point {288.611, 1713.19}),
    FrenetCart(Frenet {6168.64, 6.00452}, Point {297.046, 1691.25}),
    FrenetCart(Frenet {6192.17, 6.25228}, Point {305.69, 1669.36}),
    FrenetCart(Frenet {6215.37, 6.09894}, Point {314.941, 1647.65}),
    FrenetCart(Frenet {6239.11, 6.25051}, Point {325.198, 1626.24}),
    FrenetCart(Frenet {6262.92, 6.08167}, Point {336.063, 1604.82}),
    FrenetCart(Frenet {6265.43, 6.10247}, Point {337.218, 1602.59}),
    FrenetCart(Frenet {6309.91, 6.00823}, Point {358.135, 1563.26}),
    FrenetCart(Frenet {6332.87, 6.16583}, Point {368.942, 1543.01}),
    FrenetCart(Frenet {6356.36, 6.0073}, Point {380.286, 1522.43}),
    FrenetCart(Frenet {6379.72, 6.12178}, Point {392.237, 1502.09}),
    FrenetCart(Frenet {6403.18, 5.99355}, Point {404.339, 1482}),
    FrenetCart(Frenet {6426.66, 5.92519}, Point {416.254, 1461.84}),
    FrenetCart(Frenet {6450.01, 5.86381}, Point {428.001, 1441.66}),
    FrenetCart(Frenet {6473.46, 5.93672}, Point {439.374, 1421.37}),
    FrenetCart(Frenet {6496.78, 6.08078}, Point {450.317, 1400.77}),
    FrenetCart(Frenet {6520.04, 6.00084}, Point {461.527, 1380.16}),
    FrenetCart(Frenet {6543.47, 6.29647}, Point {473.246, 1359.82}),
    FrenetCart(Frenet {6567.07, 6.19479}, Point {485.375, 1339.57}),
    FrenetCart(Frenet {6590.19, 6.25762}, Point {497.779, 1319.73}),
    FrenetCart(Frenet {6614.27, 6.38936}, Point {510.921, 1299.54}),
    FrenetCart(Frenet {6636.67, 6.10104}, Point {523.884, 1280.76}),
    FrenetCart(Frenet {6638.17, 6.15537}, Point {524.752, 1279.54}),
    FrenetCart(Frenet {6684.42, 6.18259}, Point {552.818, 1242.77}),
    FrenetCart(Frenet {6685.95, 6.08086}, Point {553.828, 1241.62}),
    FrenetCart(Frenet {6731.19, 6.16969}, Point {587.071, 1209.55}),
    FrenetCart(Frenet {6754.11, 6.63935}, Point {604.661, 1194.14}),
    FrenetCart(Frenet {6778.88, 6.28813}, Point {624.076, 1178.75}),
    FrenetCart(Frenet {6801.17, 6.88963}, Point {643.755, 1166.18}),
    FrenetCart(Frenet {6825.4, 6.19704}, Point {665.215, 1154.92}),
    FrenetCart(Frenet {6827.42, 6.06199}, Point {667.045, 1154.04}),
    FrenetCart(Frenet {6871.68, 6.04153}, Point {710.323, 1137.52}),
    FrenetCart(Frenet {6895.8, 6.79976}, Point {733.981, 1132.76}),
    FrenetCart(Frenet {6919.46, 6.28796}, Point {758.347, 1129.99}),
  };

  for (const FrenetCart& position : positions) {
    test_convert(position.getXY(coordsConverter),
                 position.getFrenet(coordsConverter));
  }
}
