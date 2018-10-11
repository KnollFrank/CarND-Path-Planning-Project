#ifndef TESTS_COORDSCONVERTERTEST_H_
#define TESTS_COORDSCONVERTERTEST_H_

#include "../coords/coordsConverter.h"

#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include <vector>

#include "../coords/cart.h"
#include "../coords/frenet.h"
#include "../coords/waypoints.h"
#include "gtestHelper.h"

TEST(CoordsConverterTest, should_convert) {
  // GIVEN
  MapWaypoints mapWaypoints = MapWaypoints::load();
  CoordsConverter coordsConverter(mapWaypoints);

  auto test_convert =
      [&](const Point& point, const Frenet& frenet) {
        const double abs_error = 2.26;
        expect_near(frenet,
            coordsConverter.getFrenet(point),
            abs_error,
            asString([&](stringstream& stream) {
                  stream << frenet << " == coordsConverter.getFrenet(" << point <<")";}));
        expect_near(
            point,
            coordsConverter.getXY(frenet),
            abs_error,
            asString([&](stringstream& stream) {
                  stream << point << " == coordsConverter.getXY(" << frenet <<")";}));
      };

  // WHEN & THEN
  test_convert(Point { 909.48, 1128.67 }, Frenet { 124.834, 6.16483 });

  // FIXME: int i = 0 (d.h. mapWaypoints.map_waypoints_s[0] == 0) ergibt Fehler!
  for (int i = 1; i < mapWaypoints.map_waypoints.size(); i++) {
    test_convert(mapWaypoints.map_waypoints[i],
                 Frenet { mapWaypoints.map_waypoints_s[i], 0 });
  }

  vector<tuple<Frenet, Point>> frenetPointTuples = { make_tuple(Frenet {
                                                                    124.834,
                                                                    6.16483 },
                                                                Point { 909.48,
                                                                    1128.67 }),
      make_tuple(Frenet { 124.864, 6.20416 }, Point { 909.51, 1128.63 }),
      make_tuple(Frenet { 125.094, 6.20712 }, Point { 909.741, 1128.63 }),
      make_tuple(Frenet { 125.2, 6.20902 }, Point { 909.847, 1128.63 }),
      make_tuple(Frenet { 125.307, 6.20994 }, Point { 909.953, 1128.63 }),
      make_tuple(Frenet { 125.413, 6.21183 }, Point { 910.06, 1128.63 }),
      make_tuple(Frenet { 125.519, 6.21275 }, Point { 910.166, 1128.63 }),
      make_tuple(Frenet { 125.643, 6.21492 }, Point { 910.29, 1128.63 }),
      make_tuple(Frenet { 125.821, 6.21646 }, Point { 910.467, 1128.63 }),
      make_tuple(Frenet { 126.051, 6.21845 }, Point { 910.698, 1128.63 }),
      make_tuple(Frenet { 126.477, 6.22213 }, Point { 911.124, 1128.63 }),
      make_tuple(Frenet { 126.868, 6.22551 }, Point { 911.515, 1128.63 }),
      make_tuple(Frenet { 127.544, 6.23136 }, Point { 912.191, 1128.63 }),
      make_tuple(Frenet { 128.31, 6.23798 }, Point { 912.957, 1128.63 }),
      make_tuple(Frenet { 129.22, 6.24584 }, Point { 913.867, 1128.63 }),
      make_tuple(Frenet { 130.436, 6.25636 }, Point { 915.083, 1128.63 }),
      make_tuple(Frenet { 131.691, 6.2672 }, Point { 916.338, 1128.63 }),
      make_tuple(Frenet { 132.876, 6.27745 }, Point { 917.523, 1128.63 }),
      make_tuple(Frenet { 134.155, 6.28141 }, Point { 918.802, 1128.63 }),
      make_tuple(Frenet { 135.309, 6.28542 }, Point { 919.956, 1128.64 }),
      make_tuple(Frenet { 136.503, 6.28779 }, Point { 921.15, 1128.65 }),
      make_tuple(Frenet { 137.771, 6.28569 }, Point { 922.418, 1128.66 }),
      make_tuple(Frenet { 139.114, 6.28034 }, Point { 923.761, 1128.68 }),
      make_tuple(Frenet { 140.516, 6.2684 }, Point { 925.162, 1128.7 }),
      make_tuple(Frenet { 142.578, 6.23923 }, Point { 927.224, 1128.75 }),
      make_tuple(Frenet { 144.775, 6.19425 }, Point { 929.42, 1128.81 }),
      make_tuple(Frenet { 147.126, 6.12155 }, Point { 931.771, 1128.91 }),
      make_tuple(Frenet { 149.633, 6.01322 }, Point { 934.277, 1129.04 }),
      make_tuple(Frenet { 151.301, 6.01484 }, Point { 936.542, 1129.18 }),
      make_tuple(Frenet { 154.512, 6.14095 }, Point { 939.748, 1129.4 }),
      make_tuple(Frenet { 157.858, 6.23857 }, Point { 943.085, 1129.67 }),
      make_tuple(Frenet { 160.791, 6.29998 }, Point { 946.007, 1129.93 }),
      make_tuple(Frenet { 164.149, 6.3415 }, Point { 949.349, 1130.25 }),
      make_tuple(Frenet { 167.089, 6.34751 }, Point { 952.273, 1130.56 }),
      make_tuple(Frenet { 170.033, 6.32458 }, Point { 955.196, 1130.91 }),
      make_tuple(Frenet { 173.398, 6.26128 }, Point { 958.535, 1131.34 }),
      make_tuple(Frenet { 176.344, 6.16726 }, Point { 961.453, 1131.75 }),
      make_tuple(Frenet { 179.71, 6.0135 }, Point { 964.782, 1132.27 }),
      make_tuple(Frenet { 181.93, 6.06663 }, Point { 967.661, 1132.76 }),
      make_tuple(Frenet { 185.26, 6.23491 }, Point { 970.944, 1133.34 }),
      make_tuple(Frenet { 188.188, 6.36219 }, Point { 973.827, 1133.87 }),
      make_tuple(Frenet { 191.127, 6.4626 }, Point { 976.715, 1134.43 }),
      make_tuple(Frenet { 193.656, 6.5208 }, Point { 979.192, 1134.94 }),
      make_tuple(Frenet { 197.038, 6.55528 }, Point { 982.496, 1135.66 }),
      make_tuple(Frenet { 200.005, 6.54494 }, Point { 985.386, 1136.33 }),
      make_tuple(Frenet { 202.978, 6.48876 }, Point { 988.271, 1137.05 }),
      make_tuple(Frenet { 205.956, 6.38127 }, Point { 991.149, 1137.83 }),
      make_tuple(Frenet { 209.363, 6.18754 }, Point { 994.426, 1138.78 }),
      make_tuple(Frenet { 211.754, 6.02008 }, Point { 997.697, 1139.81 }),
      make_tuple(Frenet { 215.212, 6.23402 }, Point { 1000.97, 1140.95 }),
      make_tuple(Frenet { 218.215, 6.34185 }, Point { 1003.78, 1142.01 }),
      make_tuple(Frenet { 221.199, 6.386 }, Point { 1006.55, 1143.13 }),
      make_tuple(Frenet { 225.002, 6.37078 }, Point { 1010.05, 1144.61 }),
      make_tuple(Frenet { 228.352, 6.3131 }, Point { 1013.12, 1145.96 }),
      make_tuple(Frenet { 231.258, 6.24236 }, Point { 1015.77, 1147.15 }),
      make_tuple(Frenet { 233.32, 6.18678 }, Point { 1017.65, 1148 }),
      make_tuple(Frenet { 236.589, 6.1017 }, Point { 1020.63, 1149.35 }),
      make_tuple(Frenet { 239.825, 6.03811 }, Point { 1023.59, 1150.66 }),
      make_tuple(Frenet { 242.626, 6.01334 }, Point { 1026.16, 1151.76 }),
      make_tuple(Frenet { 245.918, 6.00953 }, Point { 1029.15, 1153 }),
      make_tuple(Frenet { 248.78, 6.00279 }, Point { 1031.79, 1154.09 }),
      make_tuple(Frenet { 252.055, 6.00052 }, Point { 1034.83, 1155.33 }),
      make_tuple(Frenet { 254.513, 6.00082 }, Point { 1037.1, 1156.26 }),
      make_tuple(Frenet { 257.794, 6.00212 }, Point { 1040.14, 1157.5 }),
      make_tuple(Frenet { 261.077, 6.00412 }, Point { 1043.18, 1158.73 }),
      make_tuple(Frenet { 263.131, 6.00596 }, Point { 1045.08, 1159.51 }),
      make_tuple(Frenet { 266.421, 6.00973 }, Point { 1048.13, 1160.75 }),
      make_tuple(Frenet { 269.713, 6.01243 }, Point { 1051.18, 1161.99 }),
      make_tuple(Frenet { 273.422, 6.01112 }, Point { 1054.61, 1163.39 }),
      make_tuple(Frenet { 275.849, 6.0051 }, Point { 1056.9, 1164.34 }),
      make_tuple(Frenet { 277.967, 6.00105 }, Point { 1058.86, 1165.16 }),
      make_tuple(Frenet { 280.491, 5.97015 }, Point { 1061.17, 1166.16 }),
      make_tuple(Frenet { 283.82, 5.90026 }, Point { 1064.22, 1167.51 }),
      make_tuple(Frenet { 287.518, 5.81853 }, Point { 1067.6, 1169.01 }),
      make_tuple(Frenet { 290.358, 5.76702 }, Point { 1070.2, 1170.15 }),
      make_tuple(Frenet { 292.765, 5.74393 }, Point { 1072.41, 1171.11 }),
      make_tuple(Frenet { 295.147, 5.74587 }, Point { 1074.61, 1172.02 }),
      make_tuple(Frenet { 298.282, 5.80074 }, Point { 1077.52, 1173.18 }),
      make_tuple(Frenet { 300.986, 5.91574 }, Point { 1080.06, 1174.12 }),
      make_tuple(Frenet { 304.209, 5.8895 }, Point { 1082.25, 1174.86 }),
      make_tuple(Frenet { 306.935, 5.72662 }, Point { 1084.86, 1175.68 }),
      make_tuple(Frenet { 310.467, 5.60572 }, Point { 1088.26, 1176.65 }),
      make_tuple(Frenet { 313.631, 5.57232 }, Point { 1091.32, 1177.45 }),
      make_tuple(Frenet { 317.209, 5.61127 }, Point { 1094.8, 1178.27 }),
      make_tuple(Frenet { 320.405, 5.70629 }, Point { 1097.92, 1178.95 }),
      make_tuple(Frenet { 324.009, 5.8732 }, Point { 1101.46, 1179.66 }),
      make_tuple(Frenet { 327.484, 5.91759 }, Point { 1104.21, 1180.17 }),
      make_tuple(Frenet { 330.254, 5.80846 }, Point { 1106.94, 1180.63 }),
      make_tuple(Frenet { 333.438, 5.73291 }, Point { 1110.09, 1181.12 }),
      make_tuple(Frenet { 335.839, 5.7099 }, Point { 1112.47, 1181.45 }),
      make_tuple(Frenet { 339.054, 5.71698 }, Point { 1115.66, 1181.86 }),
      make_tuple(Frenet { 341.476, 5.74417 }, Point { 1118.06, 1182.14 }),
      make_tuple(Frenet { 343.907, 5.79153 }, Point { 1120.48, 1182.41 }),
      make_tuple(Frenet { 346.345, 5.85108 }, Point { 1122.9, 1182.66 }),
      make_tuple(Frenet { 348.383, 5.90597 }, Point { 1124.93, 1182.87 }),
      make_tuple(Frenet { 351.968, 5.98067 }, Point { 1128.18, 1183.2 }),
      make_tuple(Frenet { 354.806, 5.92833 }, Point { 1131.01, 1183.46 }),
      make_tuple(Frenet { 357.243, 5.89265 }, Point { 1133.43, 1183.68 }),
      make_tuple(Frenet { 360.088, 5.86289 }, Point { 1136.27, 1183.93 }),
      make_tuple(Frenet { 363.753, 5.84513 }, Point { 1139.92, 1184.22 }),
      make_tuple(Frenet { 367.424, 5.8419 }, Point { 1143.58, 1184.51 }),
      make_tuple(Frenet { 369.464, 5.84534 }, Point { 1145.62, 1184.66 }),
      make_tuple(Frenet { 371.507, 5.85398 }, Point { 1147.66, 1184.8 }),
      make_tuple(Frenet { 373.551, 5.86867 }, Point { 1149.69, 1184.94 }),
      make_tuple(Frenet { 375.597, 5.88558 }, Point { 1151.74, 1185.08 }),
      make_tuple(Frenet { 379.693, 5.92472 }, Point { 1155.82, 1185.35 }),
      make_tuple(Frenet { 382.974, 5.9591 }, Point { 1159.1, 1185.57 }),
      make_tuple(Frenet { 387.61, 5.94069 }, Point { 1163.56, 1185.84 }),
      make_tuple(Frenet { 391.26, 5.92468 }, Point { 1167.2, 1186.02 }),
      make_tuple(Frenet { 395.335, 5.93267 }, Point { 1171.27, 1186.21 }),
      make_tuple(Frenet { 399.021, 5.94846 }, Point { 1174.95, 1186.37 }),
      make_tuple(Frenet { 403.549, 5.97292 }, Point { 1179.48, 1186.55 }),
      make_tuple(Frenet { 408.507, 5.97744 }, Point { 1184.42, 1186.78 }),
      make_tuple(Frenet { 412.208, 5.97556 }, Point { 1188.12, 1186.95 }),
      make_tuple(Frenet { 416.315, 5.97241 }, Point { 1192.22, 1187.15 }),
      make_tuple(Frenet { 421.236, 5.96592 }, Point { 1197.14, 1187.38 }),
      make_tuple(Frenet { 425.741, 5.97214 }, Point { 1201.64, 1187.58 }),
      make_tuple(Frenet { 429.42, 5.98624 }, Point { 1205.32, 1187.74 }),
      make_tuple(Frenet { 434.422, 5.99557 }, Point { 1210.24, 1187.94 }),
      make_tuple(Frenet { 438.55, 5.95291 }, Point { 1214.36, 1188.12 }),
      make_tuple(Frenet { 442.656, 5.90346 }, Point { 1218.46, 1188.31 }),
      make_tuple(Frenet { 447.142, 5.85992 }, Point { 1222.95, 1188.5 }),
      make_tuple(Frenet { 450.788, 5.84701 }, Point { 1226.59, 1188.64 }),
      make_tuple(Frenet { 455.214, 5.87743 }, Point { 1231.02, 1188.76 }),
      make_tuple(Frenet { 459.211, 5.9683 }, Point { 1235.01, 1188.8 }),
      make_tuple(Frenet { 463.634, 5.902 }, Point { 1239.04, 1188.78 }),
      make_tuple(Frenet { 467.279, 5.84469 }, Point { 1242.69, 1188.72 }),
      make_tuple(Frenet { 470.526, 5.82195 }, Point { 1245.93, 1188.64 }),
      make_tuple(Frenet { 473.779, 5.81318 }, Point { 1249.19, 1188.54 }),
      make_tuple(Frenet { 477.858, 5.82398 }, Point { 1253.26, 1188.4 }),
      make_tuple(Frenet { 480.718, 5.84364 }, Point { 1256.12, 1188.29 }),
      make_tuple(Frenet { 484.404, 5.87699 }, Point { 1259.8, 1188.14 }),
      make_tuple(Frenet { 488.92, 5.92584 }, Point { 1264.32, 1187.95 }),
      make_tuple(Frenet { 493.034, 5.97151 }, Point { 1268.43, 1187.77 }),
      make_tuple(Frenet { 497.651, 5.98868 }, Point { 1272.96, 1187.59 }),
      make_tuple(Frenet { 501.772, 5.95699 }, Point { 1277.08, 1187.43 }),
      make_tuple(Frenet { 505.884, 5.91561 }, Point { 1281.19, 1187.29 }),
      make_tuple(Frenet { 509.982, 5.87299 }, Point { 1285.29, 1187.15 }),
      make_tuple(Frenet { 514.068, 5.84377 }, Point { 1289.37, 1186.99 }),
      make_tuple(Frenet { 518.953, 5.83163 }, Point { 1294.25, 1186.79 }),
      make_tuple(Frenet { 523.415, 5.85067 }, Point { 1298.71, 1186.57 }),
      make_tuple(Frenet { 527.457, 5.90361 }, Point { 1302.74, 1186.33 }),
      make_tuple(Frenet { 532.278, 5.97392 }, Point { 1307.16, 1186.02 }),
      make_tuple(Frenet { 535.883, 5.88825 }, Point { 1310.75, 1185.7 }),
      make_tuple(Frenet { 539.912, 5.85092 }, Point { 1314.76, 1185.29 }),
      make_tuple(Frenet { 543.964, 5.8583 }, Point { 1318.78, 1184.84 }),
      make_tuple(Frenet { 548.859, 5.90122 }, Point { 1323.64, 1184.25 }),
      make_tuple(Frenet { 553.375, 5.95831 }, Point { 1328.12, 1183.69 }),
      make_tuple(Frenet { 557.919, 6.00527 }, Point { 1332.63, 1183.14 }),
      make_tuple(Frenet { 562.49, 6.01809 }, Point { 1337.18, 1182.62 }),
      make_tuple(Frenet { 566.536, 6.01846 }, Point { 1341.32, 1182.19 }),
      make_tuple(Frenet { 571.088, 6.04491 }, Point { 1345.85, 1181.74 }),
      make_tuple(Frenet { 574.804, 6.04672 }, Point { 1349.55, 1181.4 }),
      make_tuple(Frenet { 579.334, 6.03093 }, Point { 1354.06, 1181.01 }),
      make_tuple(Frenet { 583.443, 6.01374 }, Point { 1358.15, 1180.65 }),
      make_tuple(Frenet { 587.132, 6.00178 }, Point { 1361.83, 1180.32 }),
      make_tuple(Frenet { 590.814, 5.99359 }, Point { 1365.5, 1179.99 }),
      make_tuple(Frenet { 594.526, 5.99993 }, Point { 1369.16, 1179.65 }),
      make_tuple(Frenet { 597.823, 5.98948 }, Point { 1372.44, 1179.34 }),
      make_tuple(Frenet { 601.114, 5.9705 }, Point { 1375.72, 1179.04 }),
      make_tuple(Frenet { 605.627, 5.93908 }, Point { 1380.21, 1178.63 }),
      make_tuple(Frenet { 609.31, 5.91818 }, Point { 1383.88, 1178.3 }),
      make_tuple(Frenet { 613.393, 5.90546 }, Point { 1387.95, 1177.91 }),
      make_tuple(Frenet { 617.871, 5.92373 }, Point { 1392.4, 1177.46 }),
      make_tuple(Frenet { 621.119, 5.95413 }, Point { 1395.63, 1177.12 }),
      make_tuple(Frenet { 624.968, 5.96418 }, Point { 1399.26, 1176.7 }),
      make_tuple(Frenet { 628.216, 5.93607 }, Point { 1402.48, 1176.31 }),
      make_tuple(Frenet { 632.288, 5.92812 }, Point { 1406.52, 1175.78 }),
      make_tuple(Frenet { 636.779, 5.94062 }, Point { 1410.97, 1175.19 }),
      make_tuple(Frenet { 640.874, 5.95973 }, Point { 1415.03, 1174.63 }),
      make_tuple(Frenet { 644.57, 5.98294 }, Point { 1418.69, 1174.13 }),
      make_tuple(Frenet { 647.45, 5.99815 }, Point { 1421.54, 1173.74 }),
      make_tuple(Frenet { 651.575, 6.00936 }, Point { 1425.63, 1173.19 }),
      make_tuple(Frenet { 655.673, 6.001 }, Point { 1429.73, 1172.66 }),
      make_tuple(Frenet { 660.199, 6.00669 }, Point { 1434.22, 1172.09 }),
      make_tuple(Frenet { 664.31, 6.01095 }, Point { 1438.3, 1171.58 }),
      make_tuple(Frenet { 668.831, 6.0152 }, Point { 1442.78, 1171.01 }),
      make_tuple(Frenet { 673.761, 6.01146 }, Point { 1447.67, 1170.4 }),
      make_tuple(Frenet { 677.458, 6.00846 }, Point { 1451.34, 1169.94 }),
      make_tuple(Frenet { 681.921, 6.0048 }, Point { 1455.77, 1169.39 }),
      make_tuple(Frenet { 685.044, 6.00228 }, Point { 1458.87, 1169 }),
      make_tuple(Frenet { 688.415, 5.99573 }, Point { 1462.22, 1168.59 }),
      make_tuple(Frenet { 692.357, 5.99126 }, Point { 1466.13, 1168.1 }),
      make_tuple(Frenet { 695.438, 5.98869 }, Point { 1469.17, 1167.71 }),
      make_tuple(Frenet { 698.997, 5.99656 }, Point { 1472.7, 1167.26 }),
      make_tuple(Frenet { 701.762, 6.00869 }, Point { 1475.45, 1166.9 }),
      make_tuple(Frenet { 704.944, 6.02417 }, Point { 1478.6, 1166.48 }),
      make_tuple(Frenet { 707.99, 6.03186 }, Point { 1481.62, 1166.09 }),
      make_tuple(Frenet { 711.582, 6.03362 }, Point { 1485.18, 1165.63 }),
      make_tuple(Frenet { 715.381, 6.02306 }, Point { 1488.95, 1165.16 }),
      make_tuple(Frenet { 718.514, 5.99756 }, Point { 1492.21, 1164.78 }),
      make_tuple(Frenet { 722.301, 6.04095 }, Point { 1495.98, 1164.35 }),
      make_tuple(Frenet { 725.855, 6.06773 }, Point { 1499.51, 1163.96 }),
      make_tuple(Frenet { 729.158, 6.07897 }, Point { 1502.8, 1163.62 }),
      make_tuple(Frenet { 732.766, 6.08417 }, Point { 1506.38, 1163.25 }),
      make_tuple(Frenet { 736.626, 6.07973 }, Point { 1510.22, 1162.86 }),
      make_tuple(Frenet { 740.288, 6.05933 }, Point { 1513.87, 1162.51 }),
      make_tuple(Frenet { 743.771, 6.02614 }, Point { 1517.34, 1162.19 }),
      make_tuple(Frenet { 747.242, 6.01666 }, Point { 1520.95, 1161.87 }),
      make_tuple(Frenet { 750.582, 6.0358 }, Point { 1524.28, 1161.6 }),
      make_tuple(Frenet { 753.498, 6.03114 }, Point { 1527.18, 1161.38 }),
      make_tuple(Frenet { 756.262, 6.01806 }, Point { 1529.94, 1161.18 }),
      make_tuple(Frenet { 759.161, 6.00366 }, Point { 1532.83, 1160.98 }),
      make_tuple(Frenet { 762.552, 5.98667 }, Point { 1536.21, 1160.73 }),
      make_tuple(Frenet { 765.736, 5.98053 }, Point { 1539.39, 1160.5 }),
      make_tuple(Frenet { 769.778, 5.98291 }, Point { 1543.42, 1160.19 }),
      make_tuple(Frenet { 772.939, 5.99823 }, Point { 1546.48, 1159.93 }),
      make_tuple(Frenet { 776.54, 5.99166 }, Point { 1550.07, 1159.61 }),
      make_tuple(Frenet { 781.028, 6.00672 }, Point { 1554.54, 1159.19 }),
      make_tuple(Frenet { 784.986, 6.03292 }, Point { 1558.48, 1158.8 }),
      make_tuple(Frenet { 789.141, 6.06114 }, Point { 1562.61, 1158.4 }),
      make_tuple(Frenet { 793.084, 6.06556 }, Point { 1566.54, 1158.03 }),
      make_tuple(Frenet { 796.464, 6.04413 }, Point { 1569.91, 1157.75 }),
      make_tuple(Frenet { 799.523, 6.03488 }, Point { 1573.13, 1157.5 }),
      make_tuple(Frenet { 802.859, 6.04898 }, Point { 1576.46, 1157.28 }),
      make_tuple(Frenet { 806.393, 6.04049 }, Point { 1579.98, 1157.07 }),
      make_tuple(Frenet { 809.82, 6.01778 }, Point { 1583.41, 1156.88 }),
      make_tuple(Frenet { 813.105, 5.99088 }, Point { 1586.69, 1156.7 }),
      make_tuple(Frenet { 816.905, 5.9669 }, Point { 1590.48, 1156.48 }),
      make_tuple(Frenet { 820.875, 5.96735 }, Point { 1594.44, 1156.24 }),
      make_tuple(Frenet { 825.17, 5.98316 }, Point { 1598.57, 1155.94 }),
      make_tuple(Frenet { 829.952, 5.93492 }, Point { 1603.34, 1155.56 }),
      make_tuple(Frenet { 833.22, 5.91105 }, Point { 1606.59, 1155.3 }),
      make_tuple(Frenet { 836.877, 5.89467 }, Point { 1610.24, 1154.99 }),
      make_tuple(Frenet { 840.779, 5.89469 }, Point { 1614.12, 1154.64 }),
      make_tuple(Frenet { 844.5, 5.91572 }, Point { 1617.83, 1154.29 }),
      make_tuple(Frenet { 848.061, 5.95122 }, Point { 1621.37, 1153.94 }),
      make_tuple(Frenet { 851.474, 5.99965 }, Point { 1624.77, 1153.59 }),
      make_tuple(Frenet { 854.833, 5.96909 }, Point { 1627.89, 1153.24 }),
      make_tuple(Frenet { 858.085, 5.96959 }, Point { 1631.12, 1152.83 }),
      make_tuple(Frenet { 860.894, 5.99054 }, Point { 1633.9, 1152.46 }),
      make_tuple(Frenet { 863.297, 6.01717 }, Point { 1636.28, 1152.14 }),
      make_tuple(Frenet { 866.476, 6.04809 }, Point { 1639.43, 1151.71 }),
      make_tuple(Frenet { 869.903, 6.06989 }, Point { 1642.83, 1151.26 }),
      make_tuple(Frenet { 873.217, 6.05453 }, Point { 1646.12, 1150.86 }),
      make_tuple(Frenet { 876.388, 5.98851 }, Point { 1649.63, 1150.5 }),
      make_tuple(Frenet { 881.228, 6.0716 }, Point { 1654.45, 1150.1 }),
      make_tuple(Frenet { 885.11, 6.08196 }, Point { 1658.33, 1149.84 }),
      make_tuple(Frenet { 889.129, 6.06449 }, Point { 1662.34, 1149.59 }),
      make_tuple(Frenet { 892.837, 6.03315 }, Point { 1666.04, 1149.38 }),
      make_tuple(Frenet { 896.522, 5.99943 }, Point { 1669.72, 1149.17 }),
      make_tuple(Frenet { 900.587, 5.97786 }, Point { 1673.78, 1148.93 }),
      make_tuple(Frenet { 904.484, 5.99729 }, Point { 1677.66, 1148.65 }),
      make_tuple(Frenet { 908.355, 5.9744 }, Point { 1681.4, 1148.36 }),
      make_tuple(Frenet { 911.224, 5.96875 }, Point { 1684.26, 1148.12 }),
      make_tuple(Frenet { 915.647, 5.97586 }, Point { 1688.66, 1147.73 }),
      make_tuple(Frenet { 918.212, 5.98453 }, Point { 1691.22, 1147.5 }),
      make_tuple(Frenet { 920.961, 5.99624 }, Point { 1693.95, 1147.25 }),
      make_tuple(Frenet { 922.999, 6.00633 }, Point { 1695.98, 1147.06 }),
      make_tuple(Frenet { 925.04, 6.0152 }, Point { 1698.02, 1146.88 }),
      make_tuple(Frenet { 927.515, 6.02252 }, Point { 1700.48, 1146.66 }),
      make_tuple(Frenet { 935.166, 6.02745 }, Point { 1708.17, 1146.02 }),
      make_tuple(Frenet { 938.455, 6.03696 }, Point { 1711.45, 1145.76 }),
      make_tuple(Frenet { 940.37, 6.04061 }, Point { 1713.36, 1145.61 }),
      make_tuple(Frenet { 947.281, 6.0413 }, Point { 1720.25, 1145.08 }),
      make_tuple(Frenet { 951.816, 6.03019 }, Point { 1724.77, 1144.75 }),
      make_tuple(Frenet { 960.45, 5.99331 }, Point { 1733.5, 1144.14 }),
      make_tuple(Frenet { 965.983, 6.29556 }, Point { 1739, 1143.53 }),
      make_tuple(Frenet { 972.843, 7.0908 }, Point { 1745.81, 1142.34 }),
      make_tuple(Frenet { 976.355, 7.52672 }, Point { 1749.29, 1141.7 }),
      make_tuple(Frenet { 979.084, 7.86227 }, Point { 1751.99, 1141.21 }),
      make_tuple(Frenet { 980.995, 8.09141 }, Point { 1753.89, 1140.88 }),
      make_tuple(Frenet { 983.596, 8.38121 }, Point { 1756.47, 1140.44 }),
      make_tuple(Frenet { 986.159, 8.64419 }, Point { 1759.01, 1140.03 }),
      make_tuple(Frenet { 988.984, 8.90516 }, Point { 1761.82, 1139.61 }),
      make_tuple(Frenet { 992.85, 9.20052 }, Point { 1765.59, 1139.08 }),
      make_tuple(Frenet { 996.101, 9.41329 }, Point { 1768.82, 1138.65 }),
      make_tuple(Frenet { 999.09, 9.58908 }, Point { 1771.79, 1138.29 }),
      make_tuple(Frenet { 1001.56, 9.71946 }, Point { 1774.25, 1137.99 }),
      make_tuple(Frenet { 1003.6, 9.81699 }, Point { 1776.27, 1137.76 }),
      make_tuple(Frenet { 1005.97, 9.91093 }, Point { 1778.63, 1137.52 }),
      make_tuple(Frenet { 1008.4, 9.9865 }, Point { 1781.06, 1137.28 }),
      make_tuple(Frenet { 1010.04, 10.0274 }, Point { 1782.68, 1137.13 }),
      make_tuple(Frenet { 1012.15, 10.047 }, Point { 1784.79, 1136.98 }),
      make_tuple(Frenet { 1014.47, 9.96677 }, Point { 1787.11, 1136.91 }),
      make_tuple(Frenet { 1017.01, 9.77974 }, Point { 1789.66, 1136.93 }),
      make_tuple(Frenet { 1019.29, 9.51193 }, Point { 1792.37, 1137.05 }),
      make_tuple(Frenet { 1022.39, 9.24469 }, Point { 1795.48, 1137.25 }),
      make_tuple(Frenet { 1025.44, 8.95552 }, Point { 1798.54, 1137.48 }),
      make_tuple(Frenet { 1028.7, 8.63714 }, Point { 1801.8, 1137.73 }),
      make_tuple(Frenet { 1032.52, 8.26203 }, Point { 1805.62, 1138.03 }),
      make_tuple(Frenet { 1036.58, 7.86087 }, Point { 1809.69, 1138.34 }),
      make_tuple(Frenet { 1040.46, 7.4715 }, Point { 1813.58, 1138.65 }),
      make_tuple(Frenet { 1044.97, 7.00105 }, Point { 1818.1, 1139.03 }),
      make_tuple(Frenet { 1048.87, 6.68541 }, Point { 1822.79, 1139.46 }),
      make_tuple(Frenet { 1053.13, 6.64094 }, Point { 1827.03, 1139.92 }),
      make_tuple(Frenet { 1057.36, 6.54774 }, Point { 1831.22, 1140.43 }),
      make_tuple(Frenet { 1061.13, 6.43782 }, Point { 1834.97, 1140.9 }),
      make_tuple(Frenet { 1065.3, 6.29642 }, Point { 1839.1, 1141.45 }),
      make_tuple(Frenet { 1069.43, 6.15095 }, Point { 1843.2, 1141.99 }),
      make_tuple(Frenet { 1073.53, 6.02846 }, Point { 1847.27, 1142.51 }),
      make_tuple(Frenet { 1077.41, 6.00815 }, Point { 1851.32, 1143 }),
      make_tuple(Frenet { 1081.52, 6.07215 }, Point { 1855.41, 1143.48 }),
      make_tuple(Frenet { 1085.65, 6.13246 }, Point { 1859.51, 1143.96 }),
      make_tuple(Frenet { 1089.79, 6.1778 }, Point { 1863.62, 1144.46 }),
      make_tuple(Frenet { 1093.37, 6.19666 }, Point { 1867.17, 1144.92 }),
      make_tuple(Frenet { 1097.94, 6.01122 }, Point { 1871.68, 1145.7 }),
      make_tuple(Frenet { 1101.69, 5.61412 }, Point { 1875.34, 1146.59 }),
      make_tuple(Frenet { 1105.83, 4.96843 }, Point { 1879.77, 1147.85 }),
      make_tuple(Frenet { 1109.57, 4.65672 }, Point { 1883.38, 1148.97 }),
      make_tuple(Frenet { 1113.73, 4.25068 }, Point { 1887.34, 1150.28 }),
      make_tuple(Frenet { 1117.86, 3.82702 }, Point { 1891.28, 1151.59 }),
      make_tuple(Frenet { 1122.39, 3.37986 }, Point { 1895.61, 1153.02 }),
      make_tuple(Frenet { 1126.51, 3.00986 }, Point { 1899.54, 1154.27 }),
      make_tuple(Frenet { 1130.2, 2.71045 }, Point { 1903.08, 1155.37 }),
      make_tuple(Frenet { 1134.6, 2.42861 }, Point { 1907.41, 1156.67 }),
      make_tuple(Frenet { 1138.71, 2.36662 }, Point { 1911.37, 1157.81 }),
      make_tuple(Frenet { 1142.83, 2.32896 }, Point { 1915.33, 1158.92 }),
      make_tuple(Frenet { 1146.96, 2.29808 }, Point { 1919.31, 1160.03 }),
      make_tuple(Frenet { 1151.1, 2.24389 }, Point { 1923.28, 1161.17 }),
      make_tuple(Frenet { 1154.82, 2.16022 }, Point { 1926.86, 1162.23 }),
      make_tuple(Frenet { 1158.96, 2.01931 }, Point { 1930.81, 1163.45 }),
      make_tuple(Frenet { 1162.9, 2.08318 }, Point { 1934.73, 1164.72 }),
      make_tuple(Frenet { 1167.02, 2.24092 }, Point { 1938.65, 1166.01 }),
      make_tuple(Frenet { 1171.16, 2.36126 }, Point { 1942.58, 1167.34 }),
      make_tuple(Frenet { 1175.31, 2.42109 }, Point { 1946.48, 1168.73 }),
      make_tuple(Frenet { 1179.47, 2.3957 }, Point { 1950.37, 1170.2 }),
      make_tuple(Frenet { 1183.62, 2.27255 }, Point { 1954.22, 1171.77 }),
      make_tuple(Frenet { 1187.36, 2.05914 }, Point { 1957.65, 1173.27 }),
      make_tuple(Frenet { 1191.17, 2.1732 }, Point { 1961.4, 1175.05 }),
      make_tuple(Frenet { 1194.9, 2.37694 }, Point { 1964.74, 1176.74 }),
      make_tuple(Frenet { 1199.06, 2.51171 }, Point { 1968.41, 1178.7 }),
      make_tuple(Frenet { 1203.23, 2.5522 }, Point { 1972.04, 1180.74 }),
      make_tuple(Frenet { 1207.81, 2.48712 }, Point { 1975.97, 1183.08 }),
      make_tuple(Frenet { 1211.54, 2.3498 }, Point { 1979.14, 1185.07 }),
      make_tuple(Frenet { 1215.27, 2.14463 }, Point { 1982.27, 1187.1 }),
      make_tuple(Frenet { 1218.67, 2.10244 }, Point { 1985.35, 1189.2 }),
      make_tuple(Frenet { 1222.82, 2.35865 }, Point { 1988.74, 1191.6 }),
      make_tuple(Frenet { 1226.98, 2.51837 }, Point { 1992.08, 1194.09 }),
      make_tuple(Frenet { 1230.73, 2.57692 }, Point { 1995.03, 1196.4 }),
      make_tuple(Frenet { 1234.9, 2.53844 }, Point { 1998.26, 1199.05 }),
      make_tuple(Frenet { 1239.07, 2.38785 }, Point { 2001.4, 1201.78 }),
      make_tuple(Frenet { 1243.22, 2.11906 }, Point { 2004.47, 1204.59 }),
      make_tuple(Frenet { 1247, 2.18958 }, Point { 2007.44, 1207.48 }),
      make_tuple(Frenet { 1251.15, 2.43054 }, Point { 2010.35, 1210.46 }),
      make_tuple(Frenet { 1255.73, 2.57733 }, Point { 2013.47, 1213.82 }),
      make_tuple(Frenet { 1259.49, 2.6004 }, Point { 2015.95, 1216.63 }),
      make_tuple(Frenet { 1263.66, 2.5289 }, Point { 2018.64, 1219.83 }),
      make_tuple(Frenet { 1267.83, 2.34261 }, Point { 2021.23, 1223.09 }),
      make_tuple(Frenet { 1271.97, 2.04112 }, Point { 2023.73, 1226.41 }),
      make_tuple(Frenet { 1276.18, 2.25115 }, Point { 2026.36, 1230.14 }),
      make_tuple(Frenet { 1279.92, 2.40844 }, Point { 2028.42, 1233.26 }),
      make_tuple(Frenet { 1284.07, 2.49166 }, Point { 2030.64, 1236.77 }),
      make_tuple(Frenet { 1288.22, 2.49604 }, Point { 2032.79, 1240.32 }),
      make_tuple(Frenet { 1292.36, 2.42806 }, Point { 2034.88, 1243.9 }),
      make_tuple(Frenet { 1296.5, 2.30321 }, Point { 2036.91, 1247.5 }),
      make_tuple(Frenet { 1301.04, 2.11933 }, Point { 2039.1, 1251.48 }),
      make_tuple(Frenet { 1304.97, 2.02294 }, Point { 2041.07, 1255.1 }),
      make_tuple(Frenet { 1309.1, 2.16904 }, Point { 2043, 1258.75 }),
      make_tuple(Frenet { 1312.81, 2.26079 }, Point { 2044.71, 1262.05 }),
      make_tuple(Frenet { 1316.94, 2.3202 }, Point { 2046.56, 1265.74 }),
      make_tuple(Frenet { 1321.49, 2.33447 }, Point { 2048.56, 1269.82 }),
      make_tuple(Frenet { 1325.2, 2.30973 }, Point { 2050.16, 1273.18 }),
      make_tuple(Frenet { 1328.51, 2.25172 }, Point { 2051.55, 1276.18 }),
      make_tuple(Frenet { 1332.22, 2.15371 }, Point { 2053.08, 1279.56 }),
      make_tuple(Frenet { 1335.93, 2.02515 }, Point { 2054.59, 1282.95 }),
      make_tuple(Frenet { 1339.48, 2.09078 }, Point { 2056.07, 1286.37 }),
      make_tuple(Frenet { 1344.03, 2.19042 }, Point { 2057.8, 1290.58 }),
      make_tuple(Frenet { 1348.16, 2.21165 }, Point { 2059.3, 1294.43 }),
      make_tuple(Frenet { 1351.88, 2.18843 }, Point { 2060.61, 1297.9 }),
      make_tuple(Frenet { 1355.99, 2.13596 }, Point { 2062.05, 1301.76 }),
      make_tuple(Frenet { 1360.11, 2.07191 }, Point { 2063.46, 1305.62 }),
      make_tuple(Frenet { 1364.21, 2.01699 }, Point { 2064.89, 1309.46 }),
      make_tuple(Frenet { 1368.28, 1.99344 }, Point { 2066.33, 1313.3 }),
      make_tuple(Frenet { 1372.79, 2.01941 }, Point { 2067.93, 1317.51 }),
      make_tuple(Frenet { 1376.9, 2.05095 }, Point { 2069.4, 1321.35 }),
      make_tuple(Frenet { 1381.01, 2.08367 }, Point { 2070.87, 1325.18 }),
      make_tuple(Frenet { 1384.71, 2.10756 }, Point { 2072.19, 1328.64 }),
      make_tuple(Frenet { 1388.83, 2.11714 }, Point { 2073.64, 1332.49 }),
      make_tuple(Frenet { 1393.36, 2.09848 }, Point { 2075.22, 1336.75 }),
      make_tuple(Frenet { 1397.07, 2.05925 }, Point { 2076.48, 1340.23 }),
      make_tuple(Frenet { 1400.37, 2.00002 }, Point { 2077.58, 1343.35 }),
      make_tuple(Frenet { 1404.38, 2.04995 }, Point { 2078.91, 1347.26 }),
      make_tuple(Frenet { 1408.92, 2.11535 }, Point { 2080.31, 1351.58 }),
      make_tuple(Frenet { 1413.05, 2.13487 }, Point { 2081.55, 1355.51 }),
      make_tuple(Frenet { 1416.75, 2.12346 }, Point { 2082.63, 1359.06 }),
      make_tuple(Frenet { 1420.87, 2.09361 }, Point { 2083.81, 1363 }),
      make_tuple(Frenet { 1425.39, 2.04438 }, Point { 2085.09, 1367.34 }),
      make_tuple(Frenet { 1429.49, 2.00549 }, Point { 2086.26, 1371.27 }),
      make_tuple(Frenet { 1433.16, 2.015 }, Point { 2087.33, 1374.81 }),
      make_tuple(Frenet { 1437.27, 2.04009 }, Point { 2088.5, 1378.74 }),
      make_tuple(Frenet { 1441.38, 2.05495 }, Point { 2089.67, 1382.69 }),
      make_tuple(Frenet { 1445.49, 2.05844 }, Point { 2090.82, 1386.63 }),
      make_tuple(Frenet { 1450.01, 2.05221 }, Point { 2092.08, 1390.97 }),
      make_tuple(Frenet { 1454.12, 2.03696 }, Point { 2093.22, 1394.92 }),
      make_tuple(Frenet { 1457.82, 2.01516 }, Point { 2094.23, 1398.48 }),
      make_tuple(Frenet { 1461.9, 1.99509 }, Point { 2095.36, 1402.43 }),
      make_tuple(Frenet { 1465.99, 2.04431 }, Point { 2096.49, 1406.36 }),
      make_tuple(Frenet { 1470.09, 2.12023 }, Point { 2097.66, 1410.3 }),
      make_tuple(Frenet { 1474.62, 2.20708 }, Point { 2098.94, 1414.64 }),
      make_tuple(Frenet { 1478.74, 2.27047 }, Point { 2100.1, 1418.6 }),
      make_tuple(Frenet { 1482.88, 2.29046 }, Point { 2101.22, 1422.58 }),
      make_tuple(Frenet { 1487.02, 2.24614 }, Point { 2102.28, 1426.58 }),
      make_tuple(Frenet { 1490.75, 2.1302 }, Point { 2103.16, 1430.21 }),
      make_tuple(Frenet { 1494.66, 2.07493 }, Point { 2104.04, 1434.27 }),
      make_tuple(Frenet { 1498.8, 2.22886 }, Point { 2104.82, 1438.34 }),
      make_tuple(Frenet { 1502.94, 2.29851 }, Point { 2105.51, 1442.42 }),
      make_tuple(Frenet { 1507.07, 2.30595 }, Point { 2106.15, 1446.5 }),
      make_tuple(Frenet { 1511.2, 2.26391 }, Point { 2106.73, 1450.59 }),
      make_tuple(Frenet { 1515.32, 2.1966 }, Point { 2107.29, 1454.67 }),
      make_tuple(Frenet { 1519.43, 2.11815 }, Point { 2107.84, 1458.75 }),
      make_tuple(Frenet { 1524.36, 2.03408 }, Point { 2108.5, 1463.62 }),
      make_tuple(Frenet { 1528.01, 2.00059 }, Point { 2109.01, 1467.28 }),
      make_tuple(Frenet { 1531.72, 2.01364 }, Point { 2109.51, 1470.95 }),
      make_tuple(Frenet { 1536.24, 2.00251 }, Point { 2110.1, 1475.45 }),
      make_tuple(Frenet { 1539.95, 1.97445 }, Point { 2110.56, 1479.12 }),
      make_tuple(Frenet { 1543.64, 1.94414 }, Point { 2111.02, 1482.79 }),
      make_tuple(Frenet { 1548.15, 1.91409 }, Point { 2111.59, 1487.26 }),
      make_tuple(Frenet { 1552.65, 1.90436 }, Point { 2112.17, 1491.72 }),
      make_tuple(Frenet { 1556.33, 1.92684 }, Point { 2112.68, 1495.37 }),
      make_tuple(Frenet { 1560.49, 1.97352 }, Point { 2113.29, 1499.4 }),
      make_tuple(Frenet { 1564.59, 1.92464 }, Point { 2113.94, 1503.45 }),
      make_tuple(Frenet { 1569.09, 1.89284 }, Point { 2114.68, 1507.89 }),
      make_tuple(Frenet { 1573.19, 1.88102 }, Point { 2115.37, 1511.93 }),
      make_tuple(Frenet { 1577.7, 1.88629 }, Point { 2116.14, 1516.38 }),
      make_tuple(Frenet { 1581.39, 1.89928 }, Point { 2116.78, 1520.01 }),
      make_tuple(Frenet { 1585.49, 1.92246 }, Point { 2117.51, 1524.05 }),
      make_tuple(Frenet { 1590.01, 1.94721 }, Point { 2118.3, 1528.49 }),
      make_tuple(Frenet { 1594.52, 1.97491 }, Point { 2119.1, 1532.93 }),
      make_tuple(Frenet { 1598.63, 2.00754 }, Point { 2119.83, 1536.97 }),
      make_tuple(Frenet { 1603.17, 2.00391 }, Point { 2120.65, 1541.41 }),
      make_tuple(Frenet { 1606.86, 1.98695 }, Point { 2121.32, 1545.04 }),
      make_tuple(Frenet { 1610.96, 1.97598 }, Point { 2122.08, 1549.07 }),
      make_tuple(Frenet { 1614.25, 1.96847 }, Point { 2122.68, 1552.3 }),
      make_tuple(Frenet { 1617.94, 1.96519 }, Point { 2123.37, 1555.92 }),
      make_tuple(Frenet { 1622.87, 1.97151 }, Point { 2124.3, 1560.76 }),
      make_tuple(Frenet { 1626.56, 1.97553 }, Point { 2124.99, 1564.39 }),
      make_tuple(Frenet { 1630.67, 1.97886 }, Point { 2125.76, 1568.43 }),
      make_tuple(Frenet { 1635.19, 1.98229 }, Point { 2126.61, 1572.87 }),
      make_tuple(Frenet { 1638.89, 1.98286 }, Point { 2127.3, 1576.5 }),
      make_tuple(Frenet { 1642.59, 1.98134 }, Point { 2127.99, 1580.13 }),
      make_tuple(Frenet { 1646.68, 1.97954 }, Point { 2128.76, 1584.17 }),
      make_tuple(Frenet { 1649.98, 1.98656 }, Point { 2129.36, 1587.41 }),
      make_tuple(Frenet { 1653.27, 1.98301 }, Point { 2129.95, 1590.65 }),
      make_tuple(Frenet { 1656.97, 1.96026 }, Point { 2130.59, 1594.3 }),
      make_tuple(Frenet { 1661.08, 1.92058 }, Point { 2131.29, 1598.35 }),
      make_tuple(Frenet { 1665.19, 1.87357 }, Point { 2131.99, 1602.4 }),
      make_tuple(Frenet { 1668.48, 1.83485 }, Point { 2132.54, 1605.64 }),
      make_tuple(Frenet { 1672.58, 1.79372 }, Point { 2133.24, 1609.68 }),
      make_tuple(Frenet { 1677.09, 1.76437 }, Point { 2134.02, 1614.12 }),
      make_tuple(Frenet { 1681.19, 1.75512 }, Point { 2134.75, 1618.16 }),
      make_tuple(Frenet { 1684.87, 1.75812 }, Point { 2135.41, 1621.78 }),
      make_tuple(Frenet { 1688.96, 1.79174 }, Point { 2136.18, 1625.8 }),
      make_tuple(Frenet { 1693.05, 1.85724 }, Point { 2136.98, 1629.81 }),
      make_tuple(Frenet { 1696.72, 1.95372 }, Point { 2137.74, 1633.4 }),
      make_tuple(Frenet { 1700.92, 1.96246 }, Point { 2138.62, 1637.4 }),
      make_tuple(Frenet { 1705.42, 1.90296 }, Point { 2139.61, 1641.79 }),
      make_tuple(Frenet { 1709.11, 1.87339 }, Point { 2140.43, 1645.38 }),
      make_tuple(Frenet { 1713.21, 1.84688 }, Point { 2141.36, 1649.38 }),
      make_tuple(Frenet { 1717.31, 1.84105 }, Point { 2142.3, 1653.37 }),
      make_tuple(Frenet { 1721, 1.84444 }, Point { 2143.16, 1656.96 }),
      make_tuple(Frenet { 1725.1, 1.85735 }, Point { 2144.13, 1660.94 }),
      make_tuple(Frenet { 1729.62, 1.88233 }, Point { 2145.2, 1665.33 }),
      make_tuple(Frenet { 1733.72, 1.9154 }, Point { 2146.19, 1669.31 }),
      make_tuple(Frenet { 1737.83, 1.95345 }, Point { 2147.18, 1673.3 }),
      make_tuple(Frenet { 1741.93, 1.99317 }, Point { 2148.17, 1677.28 }),
      make_tuple(Frenet { 1745.63, 2.0291 }, Point { 2149.06, 1680.87 }),
      make_tuple(Frenet { 1750.15, 2.07 }, Point { 2150.15, 1685.25 }),
      make_tuple(Frenet { 1753.85, 2.1024 }, Point { 2151.04, 1688.84 }),
      make_tuple(Frenet { 1758.37, 2.13975 }, Point { 2152.13, 1693.23 }),
      make_tuple(Frenet { 1762.48, 2.16314 }, Point { 2153.1, 1697.23 }),
      make_tuple(Frenet { 1766.18, 2.17466 }, Point { 2153.97, 1700.82 }),
      make_tuple(Frenet { 1770.7, 2.1857 }, Point { 2155.03, 1705.22 }),
      make_tuple(Frenet { 1774.4, 2.18467 }, Point { 2155.89, 1708.82 }),
      make_tuple(Frenet { 1778.92, 2.16795 }, Point { 2156.93, 1713.22 }),
      make_tuple(Frenet { 1783.04, 2.13417 }, Point { 2157.85, 1717.23 }),
      make_tuple(Frenet { 1786.99, 2.09127 }, Point { 2158.72, 1721.09 }),
      make_tuple(Frenet { 1791.11, 2.21478 }, Point { 2159.8, 1725.06 }),
      make_tuple(Frenet { 1795.06, 2.76203 }, Point { 2161.04, 1729.01 }),
      make_tuple(Frenet { 1799.98, 3.47951 }, Point { 2162.56, 1733.74 }),
      make_tuple(Frenet { 1804.49, 4.0892 }, Point { 2163.91, 1738.1 }),
      make_tuple(Frenet { 1808.19, 4.52135 }, Point { 2164.95, 1741.68 }),
      make_tuple(Frenet { 1812.72, 4.95234 }, Point { 2166.12, 1746.07 }),
      make_tuple(Frenet { 1816.43, 5.22222 }, Point { 2167.01, 1749.68 }),
      make_tuple(Frenet { 1820.14, 5.42695 }, Point { 2167.82, 1753.3 }),
      make_tuple(Frenet { 1824.24, 5.58974 }, Point { 2168.66, 1757.33 }),
      make_tuple(Frenet { 1828.34, 5.70705 }, Point { 2169.45, 1761.35 }),
      make_tuple(Frenet { 1832.43, 5.78772 }, Point { 2170.21, 1765.36 }),
      make_tuple(Frenet { 1836.01, 5.90214 }, Point { 2170.87, 1769.01 }),
      make_tuple(Frenet { 1840.55, 6.00838 }, Point { 2171.64, 1773.48 }),
      make_tuple(Frenet { 1844.67, 6.06428 }, Point { 2172.29, 1777.55 }),
      make_tuple(Frenet { 1848.79, 6.09418 }, Point { 2172.91, 1781.62 }),
      make_tuple(Frenet { 1852.9, 6.09919 }, Point { 2173.51, 1785.7 }),
      make_tuple(Frenet { 1857.01, 6.08891 }, Point { 2174.1, 1789.77 }),
      make_tuple(Frenet { 1861.12, 6.07069 }, Point { 2174.68, 1793.83 }),
      make_tuple(Frenet { 1864.82, 6.04839 }, Point { 2175.19, 1797.49 }),
      make_tuple(Frenet { 1869.33, 6.03168 }, Point { 2175.83, 1801.96 }),
      make_tuple(Frenet { 1873.02, 6.02166 }, Point { 2176.35, 1805.61 }),
      make_tuple(Frenet { 1876.69, 6.01785 }, Point { 2176.88, 1809.25 }),
      make_tuple(Frenet { 1880.37, 6.02795 }, Point { 2177.41, 1812.89 }),
      make_tuple(Frenet { 1884.46, 6.05368 }, Point { 2178.02, 1816.94 }),
      make_tuple(Frenet { 1888.57, 6.08139 }, Point { 2178.64, 1821 }),
      make_tuple(Frenet { 1892.26, 6.11013 }, Point { 2179.19, 1824.66 }),
      make_tuple(Frenet { 1896.79, 6.14304 }, Point { 2179.87, 1829.13 }),
      make_tuple(Frenet { 1900.91, 6.16074 }, Point { 2180.48, 1833.21 }),
      make_tuple(Frenet { 1905.04, 6.16437 }, Point { 2181.07, 1837.3 }),
      make_tuple(Frenet { 1909.18, 6.15291 }, Point { 2181.66, 1841.39 }),
      make_tuple(Frenet { 1913.32, 6.11626 }, Point { 2182.21, 1845.5 }),
      make_tuple(Frenet { 1917.06, 6.05502 }, Point { 2182.68, 1849.2 }),
      make_tuple(Frenet { 1920.47, 6.01464 }, Point { 2183.14, 1852.93 }),
      make_tuple(Frenet { 1925.09, 6.10817 }, Point { 2183.64, 1857.52 }),
      make_tuple(Frenet { 1928.85, 6.13012 }, Point { 2183.98, 1861.26 }),
      make_tuple(Frenet { 1933.42, 6.10358 }, Point { 2184.36, 1865.82 }),
      make_tuple(Frenet { 1937.13, 6.06105 }, Point { 2184.64, 1869.53 }),
      make_tuple(Frenet { 1941.24, 6.01061 }, Point { 2184.94, 1873.62 }),
      make_tuple(Frenet { 1945.33, 5.96213 }, Point { 2185.25, 1877.69 }),
      make_tuple(Frenet { 1949.38, 5.942 }, Point { 2185.58, 1881.74 }),
      make_tuple(Frenet { 1953.02, 5.96025 }, Point { 2185.92, 1885.36 }),
      make_tuple(Frenet { 1957.31, 5.95904 }, Point { 2186.34, 1889.37 }),
      make_tuple(Frenet { 1960.97, 5.88826 }, Point { 2186.74, 1893 }),
      make_tuple(Frenet { 1964.63, 5.84232 }, Point { 2187.18, 1896.63 }),
      make_tuple(Frenet { 1967.88, 5.82178 }, Point { 2187.58, 1899.86 }),
      make_tuple(Frenet { 1971.94, 5.82215 }, Point { 2188.11, 1903.89 }),
      make_tuple(Frenet { 1976.41, 5.85383 }, Point { 2188.73, 1908.32 }),
      make_tuple(Frenet { 1980.48, 5.91177 }, Point { 2189.32, 1912.34 }),
      make_tuple(Frenet { 1984.4, 5.97541 }, Point { 2189.87, 1915.95 }),
      make_tuple(Frenet { 1988.88, 5.90312 }, Point { 2190.59, 1920.38 }),
      make_tuple(Frenet { 1993.37, 5.85863 }, Point { 2191.33, 1924.8 }),
      make_tuple(Frenet { 1997.44, 5.83769 }, Point { 2192.02, 1928.82 }),
      make_tuple(Frenet { 2001.93, 5.83507 }, Point { 2192.81, 1933.23 }),
      make_tuple(Frenet { 2005.6, 5.84839 }, Point { 2193.46, 1936.85 }),
      make_tuple(Frenet { 2009.68, 5.87743 }, Point { 2194.21, 1940.86 }),
      make_tuple(Frenet { 2014.59, 5.93639 }, Point { 2195.12, 1945.67 }),
      make_tuple(Frenet { 2019.68, 5.96427 }, Point { 2196.06, 1950.46 }),
      make_tuple(Frenet { 2023.31, 5.93049 }, Point { 2196.79, 1954.02 }),
      make_tuple(Frenet { 2027.78, 5.9381 }, Point { 2197.75, 1958.38 }),
      make_tuple(Frenet { 2031.86, 5.97156 }, Point { 2198.64, 1962.36 }),
      make_tuple(Frenet { 2036.38, 6.01677 }, Point { 2199.64, 1966.77 }),
      make_tuple(Frenet { 2040.52, 6.04496 }, Point { 2200.55, 1970.81 }),
      make_tuple(Frenet { 2044.68, 6.04321 }, Point { 2201.43, 1974.88 }),
      make_tuple(Frenet { 2048.68, 6.01711 }, Point { 2202.26, 1978.97 }),
      make_tuple(Frenet { 2053.23, 6.05356 }, Point { 2203.12, 1983.44 }),
      make_tuple(Frenet { 2057.78, 6.06809 }, Point { 2203.96, 1987.91 }),
      make_tuple(Frenet { 2061.9, 6.06978 }, Point { 2204.71, 1991.97 }),
      make_tuple(Frenet { 2065.61, 6.06439 }, Point { 2205.38, 1995.62 }),
      make_tuple(Frenet { 2069.72, 6.05144 }, Point { 2206.11, 1999.66 }),
      make_tuple(Frenet { 2074.23, 6.03329 }, Point { 2206.91, 2004.1 }),
      make_tuple(Frenet { 2078.33, 6.02001 }, Point { 2207.64, 2008.13 }),
      make_tuple(Frenet { 2082.83, 6.01324 }, Point { 2208.45, 2012.56 }),
      make_tuple(Frenet { 2087.39, 5.99633 }, Point { 2209.27, 2017.03 }),
      make_tuple(Frenet { 2091.5, 5.96488 }, Point { 2209.99, 2021.08 }),
      make_tuple(Frenet { 2095.61, 5.92815 }, Point { 2210.71, 2025.12 }),
      make_tuple(Frenet { 2099.7, 5.8994 }, Point { 2211.43, 2029.15 }),
      make_tuple(Frenet { 2103.77, 5.88354 }, Point { 2212.17, 2033.16 }),
      make_tuple(Frenet { 2107.43, 5.89337 }, Point { 2212.85, 2036.75 }),
      make_tuple(Frenet { 2111.47, 5.93815 }, Point { 2213.63, 2040.72 }),
      make_tuple(Frenet { 2115.8, 5.96044 }, Point { 2214.47, 2044.68 }),
      make_tuple(Frenet { 2119.89, 5.88852 }, Point { 2215.33, 2048.68 }),
      make_tuple(Frenet { 2124.78, 5.81879 }, Point { 2216.38, 2053.46 }),
      make_tuple(Frenet { 2128.45, 5.78152 }, Point { 2217.19, 2057.04 }),
      make_tuple(Frenet { 2132.53, 5.76455 }, Point { 2218.1, 2061.01 }),
      make_tuple(Frenet { 2137.01, 5.77805 }, Point { 2219.14, 2065.37 }),
      make_tuple(Frenet { 2141.07, 5.81746 }, Point { 2220.11, 2069.32 }),
      make_tuple(Frenet { 2145.94, 5.90164 }, Point { 2221.31, 2074.04 }),
      make_tuple(Frenet { 2149.59, 5.98849 }, Point { 2222.23, 2077.57 }),
      make_tuple(Frenet { 2153.19, 5.92989 }, Point { 2223.07, 2080.66 }),
      make_tuple(Frenet { 2155.99, 5.87843 }, Point { 2223.84, 2083.36 }),
      make_tuple(Frenet { 2159.21, 5.85486 }, Point { 2224.76, 2086.44 }),
      make_tuple(Frenet { 2162.05, 5.85898 }, Point { 2225.59, 2089.15 }),
      make_tuple(Frenet { 2165.3, 5.88429 }, Point { 2226.57, 2092.26 }),
      make_tuple(Frenet { 2169.39, 5.92625 }, Point { 2227.8, 2096.16 }),
      make_tuple(Frenet { 2173.1, 5.96744 }, Point { 2228.93, 2099.69 }),
      make_tuple(Frenet { 2177.24, 5.99907 }, Point { 2230.17, 2103.64 }),
      make_tuple(Frenet { 2181.36, 6.00892 }, Point { 2231.39, 2107.59 }),
      make_tuple(Frenet { 2185.03, 6.02701 }, Point { 2232.47, 2111.11 }),
      make_tuple(Frenet { 2189.13, 6.05449 }, Point { 2233.68, 2115.02 }),
      make_tuple(Frenet { 2193.24, 6.08649 }, Point { 2234.9, 2118.95 }),
      make_tuple(Frenet { 2197.37, 6.11167 }, Point { 2236.12, 2122.89 }),
      make_tuple(Frenet { 2202.33, 6.11836 }, Point { 2237.56, 2127.64 }),
      make_tuple(Frenet { 2206.06, 6.10462 }, Point { 2238.63, 2131.21 }),
      make_tuple(Frenet { 2209.39, 6.06918 }, Point { 2239.56, 2134.41 }),
      make_tuple(Frenet { 2213.26, 6.01624 }, Point { 2240.68, 2138.42 }),
      make_tuple(Frenet { 2217.42, 6.09825 }, Point { 2241.77, 2142.44 }),
      make_tuple(Frenet { 2221.16, 6.13789 }, Point { 2242.71, 2146.05 }),
      make_tuple(Frenet { 2224.89, 6.15811 }, Point { 2243.63, 2149.67 }),
      make_tuple(Frenet { 2229.03, 6.16094 }, Point { 2244.63, 2153.69 }),
      make_tuple(Frenet { 2233.16, 6.14156 }, Point { 2245.61, 2157.7 }),
      make_tuple(Frenet { 2236.88, 6.10894 }, Point { 2246.48, 2161.32 }),
      make_tuple(Frenet { 2241.41, 6.05274 }, Point { 2247.52, 2165.73 }),
      make_tuple(Frenet { 2245.75, 6.03502 }, Point { 2248.55, 2170.17 }),
      make_tuple(Frenet { 2249.92, 6.06637 }, Point { 2249.44, 2174.25 }),
      make_tuple(Frenet { 2254.07, 6.06659 }, Point { 2250.29, 2178.31 }),
      make_tuple(Frenet { 2258.2, 6.04266 }, Point { 2251.12, 2182.35 }),
      make_tuple(Frenet { 2262.31, 6.01329 }, Point { 2251.93, 2186.38 }),
      make_tuple(Frenet { 2266.8, 5.99268 }, Point { 2252.84, 2190.78 }),
      make_tuple(Frenet { 2270.45, 5.9957 }, Point { 2253.59, 2194.37 }),
      make_tuple(Frenet { 2274.55, 6.02019 }, Point { 2254.45, 2198.37 }),
      make_tuple(Frenet { 2278.24, 6.04399 }, Point { 2255.23, 2201.98 }),
      make_tuple(Frenet { 2282.35, 6.075 }, Point { 2256.1, 2206 }), make_tuple(
          Frenet { 2286.9, 6.0963 }, Point { 2257.05, 2210.44 }), make_tuple(
          Frenet { 2291.04, 6.09863 }, Point { 2257.89, 2214.5 }), make_tuple(
          Frenet { 2295.2, 6.0629 }, Point { 2258.71, 2218.57 }), make_tuple(
          Frenet { 2298.84, 6.00126 }, Point { 2259.41, 2222.26 }), make_tuple(
          Frenet { 2302.87, 6.06753 }, Point { 2260.15, 2226.39 }), make_tuple(
          Frenet { 2306.63, 6.07463 }, Point { 2260.75, 2230.1 }), make_tuple(
          Frenet { 2311.18, 6.04009 }, Point { 2261.45, 2234.6 }), make_tuple(
          Frenet { 2314.89, 6.00254 }, Point { 2262, 2238.26 }), make_tuple(
          Frenet { 2318.16, 5.96883 }, Point { 2262.49, 2241.5 }), make_tuple(
          Frenet { 2321.82, 5.94637 }, Point { 2263.05, 2245.12 }), make_tuple(
          Frenet { 2325.86, 5.9509 }, Point { 2263.7, 2249.11 }), make_tuple(
          Frenet { 2329.3, 5.97853 }, Point { 2264.25, 2252.28 }), make_tuple(
          Frenet { 2333.36, 5.91891 }, Point { 2264.99, 2256.27 }), make_tuple(
          Frenet { 2337.84, 5.89017 }, Point { 2265.84, 2260.67 }), make_tuple(
          Frenet { 2341.92, 5.8871 }, Point { 2266.64, 2264.67 }), make_tuple(
          Frenet { 2346.01, 5.90295 }, Point { 2267.45, 2268.67 }), make_tuple(
          Frenet { 2350.51, 5.92894 }, Point { 2268.36, 2273.08 }), make_tuple(
          Frenet { 2355.03, 5.96225 }, Point { 2269.28, 2277.5 }), make_tuple(
          Frenet { 2359.55, 5.99667 }, Point { 2270.2, 2281.93 }), make_tuple(
          Frenet { 2364.2, 5.97162 }, Point { 2271.11, 2286.43 }), make_tuple(
          Frenet { 2368.35, 5.88791 }, Point { 2271.88, 2290.51 }), make_tuple(
          Frenet { 2372.05, 5.80164 }, Point { 2272.56, 2294.14 }), make_tuple(
          Frenet { 2376.51, 5.71817 }, Point { 2273.4, 2298.53 }), make_tuple(
          Frenet { 2380.12, 5.69598 }, Point { 2274.12, 2302.06 }), make_tuple(
          Frenet { 2383.68, 5.73501 }, Point { 2274.89, 2305.54 }), make_tuple(
          Frenet { 2387.98, 5.90313 }, Point { 2275.95, 2309.71 }), make_tuple(
          Frenet { 2392.26, 5.87169 }, Point { 2276.93, 2313.09 }), make_tuple(
          Frenet { 2396.45, 5.75386 }, Point { 2278.19, 2317.09 }), make_tuple(
          Frenet { 2400.03, 5.6153 }, Point { 2279.24, 2320.51 }), make_tuple(
          Frenet { 2403.65, 5.32763 }, Point { 2280.16, 2324.03 }), make_tuple(
          Frenet { 2407.7, 4.92387 }, Point { 2281.11, 2327.99 }), make_tuple(
          Frenet { 2411.78, 4.48621 }, Point { 2282.04, 2331.98 }), make_tuple(
          Frenet { 2415.88, 4.06622 }, Point { 2282.99, 2335.99 }), make_tuple(
          Frenet { 2420, 3.66532 }, Point { 2283.97, 2340.02 }), make_tuple(
          Frenet { 2424.14, 3.27891 }, Point { 2284.96, 2344.05 }), make_tuple(
          Frenet { 2428.2, 2.9692 }, Point { 2285.97, 2348.05 }), make_tuple(
          Frenet { 2432.3, 2.72443 }, Point { 2287.02, 2352.03 }), make_tuple(
          Frenet { 2436.41, 2.52173 }, Point { 2288.11, 2356 }), make_tuple(
          Frenet { 2440.93, 2.34294 }, Point { 2289.36, 2360.34 }), make_tuple(
          Frenet { 2445.03, 2.21344 }, Point { 2290.52, 2364.28 }), make_tuple(
          Frenet { 2449.96, 2.09714 }, Point { 2291.95, 2369 }), make_tuple(
          Frenet { 2454.07, 2.03317 }, Point { 2293.17, 2372.92 }), make_tuple(
          Frenet { 2457.76, 1.99757 }, Point { 2294.29, 2376.44 }), make_tuple(
          Frenet { 2461.85, 1.98531 }, Point { 2295.56, 2380.33 }), make_tuple(
          Frenet { 2465.95, 2.01522 }, Point { 2296.86, 2384.21 }), make_tuple(
          Frenet { 2470.46, 2.07883 }, Point { 2298.33, 2388.48 }), make_tuple(
          Frenet { 2474, 2.13284 }, Point { 2299.48, 2391.83 }), make_tuple(
          Frenet { 2477.78, 2.18542 }, Point { 2300.71, 2395.4 }), make_tuple(
          Frenet { 2481.36, 2.21093 }, Point { 2301.85, 2398.8 }), make_tuple(
          Frenet { 2484.78, 2.20774 }, Point { 2302.91, 2402.04 }), make_tuple(
          Frenet { 2487.68, 2.17988 }, Point { 2303.8, 2404.82 }), make_tuple(
          Frenet { 2491.08, 2.10136 }, Point { 2304.78, 2408.07 }), make_tuple(
          Frenet { 2494.06, 1.99143 }, Point { 2305.65, 2411.1 }), make_tuple(
          Frenet { 2497.05, 2.10912 }, Point { 2306.43, 2414 }), make_tuple(
          Frenet { 2499.66, 2.17692 }, Point { 2307.08, 2416.52 }), make_tuple(
          Frenet { 2502.75, 2.22282 }, Point { 2307.82, 2419.53 }), make_tuple(
          Frenet { 2505.99, 2.24491 }, Point { 2308.56, 2422.69 }), make_tuple(
          Frenet { 2509.06, 2.24046 }, Point { 2309.24, 2425.68 }), make_tuple(
          Frenet { 2512.63, 2.21237 }, Point { 2310.01, 2429.16 }), make_tuple(
          Frenet { 2515.39, 2.17399 }, Point { 2310.59, 2431.86 }), make_tuple(
          Frenet { 2518.27, 2.12509 }, Point { 2311.18, 2434.68 }), make_tuple(
          Frenet { 2520.97, 2.06971 }, Point { 2311.73, 2437.32 }), make_tuple(
          Frenet { 2523.81, 2.0192 }, Point { 2312.31, 2440.1 }), make_tuple(
          Frenet { 2526.84, 1.96346 }, Point { 2312.94, 2443.07 }), make_tuple(
          Frenet { 2530.32, 1.95623 }, Point { 2313.67, 2446.53 }), make_tuple(
          Frenet { 2533.74, 2.00142 }, Point { 2314.37, 2449.89 }), make_tuple(
          Frenet { 2537.34, 2.02555 }, Point { 2315.08, 2453.42 }), make_tuple(
          Frenet { 2541.12, 2.03205 }, Point { 2315.8, 2457.12 }), make_tuple(
          Frenet { 2544.66, 2.02075 }, Point { 2316.46, 2460.6 }), make_tuple(
          Frenet { 2548.79, 1.99678 }, Point { 2317.22, 2464.66 }), make_tuple(
          Frenet { 2552.36, 1.97366 }, Point { 2317.88, 2468.17 }), make_tuple(
          Frenet { 2555.38, 1.97305 }, Point { 2318.44, 2471.17 }), make_tuple(
          Frenet { 2558.62, 1.99006 }, Point { 2319.03, 2474.35 }), make_tuple(
          Frenet { 2561.37, 1.99805 }, Point { 2319.53, 2477.06 }), make_tuple(
          Frenet { 2564.25, 2.00018 }, Point { 2320.03, 2479.89 }), make_tuple(
          Frenet { 2566.95, 1.99597 }, Point { 2320.51, 2482.55 }), make_tuple(
          Frenet { 2569.79, 1.99637 }, Point { 2321.01, 2485.34 }), make_tuple(
          Frenet { 2573.47, 1.9967 }, Point { 2321.66, 2488.97 }), make_tuple(
          Frenet { 2577, 1.99647 }, Point { 2322.28, 2492.45 }), make_tuple(
          Frenet { 2580.07, 1.99637 }, Point { 2322.82, 2495.47 }), make_tuple(
          Frenet { 2582.68, 1.99855 }, Point { 2323.29, 2498.03 }), make_tuple(
          Frenet { 2585.76, 2.00041 }, Point { 2323.83, 2501.07 }), make_tuple(
          Frenet { 2588.94, 2.00466 }, Point { 2324.4, 2504.19 }), make_tuple(
          Frenet { 2592.18, 2.01846 }, Point { 2324.99, 2507.38 }), make_tuple(
          Frenet { 2594.96, 2.04266 }, Point { 2325.5, 2510.11 }), make_tuple(
          Frenet { 2597.99, 2.08221 }, Point { 2326.08, 2513.09 }), make_tuple(
          Frenet { 2601.24, 2.12946 }, Point { 2326.7, 2516.28 }), make_tuple(
          Frenet { 2604.36, 2.17361 }, Point { 2327.3, 2519.34 }), make_tuple(
          Frenet { 2607.3, 2.20643 }, Point { 2327.85, 2522.23 }), make_tuple(
          Frenet { 2610.72, 2.22801 }, Point { 2328.48, 2525.59 }), make_tuple(
          Frenet { 2614.3, 2.23217 }, Point { 2329.11, 2529.12 }), make_tuple(
          Frenet { 2618.07, 2.2086 }, Point { 2329.76, 2532.83 }), make_tuple(
          Frenet { 2622.4, 2.12905 }, Point { 2330.45, 2537.1 }), make_tuple(
          Frenet { 2626.12, 2.00652 }, Point { 2330.99, 2540.78 }), make_tuple(
          Frenet { 2629.9, 2.14835 }, Point { 2331.49, 2544.71 }), make_tuple(
          Frenet { 2634.04, 2.23474 }, Point { 2331.96, 2548.82 }), make_tuple(
          Frenet { 2637.27, 2.26314 }, Point { 2332.29, 2552.03 }), make_tuple(
          Frenet { 2640.68, 2.25517 }, Point { 2332.6, 2555.43 }), make_tuple(
          Frenet { 2644.24, 2.22472 }, Point { 2332.9, 2558.97 }), make_tuple(
          Frenet { 2646.98, 2.18362 }, Point { 2333.11, 2561.71 }), make_tuple(
          Frenet { 2649.84, 2.12894 }, Point { 2333.32, 2564.57 }), make_tuple(
          Frenet { 2652.28, 2.07127 }, Point { 2333.49, 2566.99 }), make_tuple(
          Frenet { 2655.05, 1.99943 }, Point { 2333.68, 2569.76 }), make_tuple(
          Frenet { 2657.48, 2.02089 }, Point { 2333.84, 2572.33 }), make_tuple(
          Frenet { 2660.03, 2.09376 }, Point { 2333.96, 2574.88 }), make_tuple(
          Frenet { 2662.47, 2.11751 }, Point { 2334.03, 2577.32 }), make_tuple(
          Frenet { 2665.68, 2.1009 }, Point { 2334.08, 2580.53 }), make_tuple(
          Frenet { 2669.39, 2.045 }, Point { 2334.09, 2584.24 }), make_tuple(
          Frenet { 2672.58, 2.00283 }, Point { 2334.1, 2587.49 }), make_tuple(
          Frenet { 2676.38, 2.03562 }, Point { 2334.08, 2591.3 }), make_tuple(
          Frenet { 2679.66, 2.03486 }, Point { 2334.04, 2594.57 }), make_tuple(
          Frenet { 2683.45, 2.0145 }, Point { 2333.98, 2598.36 }), make_tuple(
          Frenet { 2687.78, 1.98767 }, Point { 2333.9, 2602.7 }), make_tuple(
          Frenet { 2692.29, 1.97804 }, Point { 2333.83, 2607.2 }), make_tuple(
          Frenet { 2695.97, 1.99904 }, Point { 2333.81, 2610.88 }), make_tuple(
          Frenet { 2700.09, 1.97903 }, Point { 2333.82, 2614.95 }), make_tuple(
          Frenet { 2703.99, 1.96951 }, Point { 2333.85, 2618.85 }), make_tuple(
          Frenet { 2708.09, 1.97789 }, Point { 2333.9, 2622.95 }), make_tuple(
          Frenet { 2711.66, 1.99803 }, Point { 2333.96, 2626.52 }), make_tuple(
          Frenet { 2715.35, 2.01974 }, Point { 2334.02, 2630.22 }), make_tuple(
          Frenet { 2718.53, 2.03594 }, Point { 2334.07, 2633.4 }), make_tuple(
          Frenet { 2721.54, 2.04379 }, Point { 2334.11, 2636.4 }), make_tuple(
          Frenet { 2724.9, 2.03979 }, Point { 2334.14, 2639.78 }), make_tuple(
          Frenet { 2727.53, 2.04394 }, Point { 2334.14, 2642.42 }), make_tuple(
          Frenet { 2730.01, 2.03394 }, Point { 2334.13, 2644.89 }), make_tuple(
          Frenet { 2732.51, 2.01685 }, Point { 2334.12, 2647.4 }), make_tuple(
          Frenet { 2734.94, 1.99585 }, Point { 2334.1, 2649.82 }), make_tuple(
          Frenet { 2737.29, 1.97998 }, Point { 2334.08, 2652.17 }), make_tuple(
          Frenet { 2739.55, 1.97681 }, Point { 2334.08, 2654.43 }), make_tuple(
          Frenet { 2742.51, 1.99292 }, Point { 2334.09, 2657.39 }), make_tuple(
          Frenet { 2745.43, 1.9982 }, Point { 2334.14, 2660.24 }), make_tuple(
          Frenet { 2748.76, 1.97797 }, Point { 2334.23, 2663.58 }), make_tuple(
          Frenet { 2751.98, 1.97977 }, Point { 2334.34, 2666.8 }), make_tuple(
          Frenet { 2755.76, 1.99173 }, Point { 2334.48, 2670.58 }), make_tuple(
          Frenet { 2759.75, 2.0049 }, Point { 2334.62, 2674.56 }), make_tuple(
          Frenet { 2763.56, 2.00471 }, Point { 2334.75, 2678.37 }), make_tuple(
          Frenet { 2767.96, 1.9692 }, Point { 2334.86, 2682.77 }), make_tuple(
          Frenet { 2771.62, 1.99358 }, Point { 2334.92, 2686.47 }), make_tuple(
          Frenet { 2776.14, 2.00622 }, Point { 2334.97, 2691 }), make_tuple(
          Frenet { 2780.24, 1.99702 }, Point { 2334.99, 2695.1 }), make_tuple(
          Frenet { 2783.78, 1.98497 }, Point { 2335, 2698.63 }), make_tuple(
          Frenet { 2787.53, 1.97547 }, Point { 2335.02, 2702.38 }), make_tuple(
          Frenet { 2791.45, 1.97074 }, Point { 2335.05, 2706.31 }), make_tuple(
          Frenet { 2794.84, 1.97771 }, Point { 2335.08, 2709.69 }), make_tuple(
          Frenet { 2798.05, 1.98692 }, Point { 2335.12, 2712.9 }), make_tuple(
          Frenet { 2801.06, 2.00328 }, Point { 2335.17, 2715.91 }), make_tuple(
          Frenet { 2803.63, 2.01725 }, Point { 2335.21, 2718.48 }), make_tuple(
          Frenet { 2806.31, 2.03202 }, Point { 2335.26, 2721.16 }), make_tuple(
          Frenet { 2808.59, 2.03912 }, Point { 2335.29, 2723.43 }), make_tuple(
          Frenet { 2811.5, 2.03844 }, Point { 2335.32, 2726.35 }), make_tuple(
          Frenet { 2814.32, 2.03629 }, Point { 2335.33, 2729.2 }), make_tuple(
          Frenet { 2817.04, 2.0433 }, Point { 2335.33, 2731.92 }), make_tuple(
          Frenet { 2820.24, 2.04186 }, Point { 2335.31, 2735.13 }), make_tuple(
          Frenet { 2823.62, 2.03296 }, Point { 2335.28, 2738.5 }), make_tuple(
          Frenet { 2827.55, 2.01792 }, Point { 2335.25, 2742.43 }), make_tuple(
          Frenet { 2831.29, 2.01199 }, Point { 2335.23, 2746.17 }), make_tuple(
          Frenet { 2835.24, 2.0136 }, Point { 2335.22, 2750.09 }), make_tuple(
          Frenet { 2839.31, 2.04567 }, Point { 2335.29, 2754.17 }), make_tuple(
          Frenet { 2843.4, 2.12235 }, Point { 2335.4, 2758.25 }), make_tuple(
          Frenet { 2847.34, 2.19432 }, Point { 2335.5, 2762.2 }), make_tuple(
          Frenet { 2850.75, 2.21783 }, Point { 2335.56, 2765.6 }), make_tuple(
          Frenet { 2854.75, 2.15753 }, Point { 2335.53, 2769.6 }), make_tuple(
          Frenet { 2858.2, 1.99366 }, Point { 2335.39, 2773.05 }), make_tuple(
          Frenet { 2860.86, 2.13712 }, Point { 2335.21, 2775.96 }), make_tuple(
          Frenet { 2863.94, 2.25486 }, Point { 2334.97, 2779.03 }), make_tuple(
          Frenet { 2866.85, 2.33406 }, Point { 2334.71, 2781.93 }), make_tuple(
          Frenet { 2869.55, 2.36523 }, Point { 2334.43, 2784.62 }), make_tuple(
          Frenet { 2872.35, 2.36164 }, Point { 2334.11, 2787.4 }), make_tuple(
          Frenet { 2874.68, 2.32879 }, Point { 2333.81, 2789.71 }), make_tuple(
          Frenet { 2877.03, 2.2635 }, Point { 2333.47, 2792.03 }), make_tuple(
          Frenet { 2879.26, 2.1745 }, Point { 2333.12, 2794.24 }), make_tuple(
          Frenet { 2881.48, 2.06479 }, Point { 2332.76, 2796.43 }), make_tuple(
          Frenet { 2883.35, 1.96175 }, Point { 2332.39, 2798.52 }), make_tuple(
          Frenet { 2886.16, 2.11103 }, Point { 2331.87, 2801.29 }), make_tuple(
          Frenet { 2888.89, 2.20488 }, Point { 2331.31, 2803.96 }), make_tuple(
          Frenet { 2891.49, 2.25679 }, Point { 2330.74, 2806.49 }), make_tuple(
          Frenet { 2894.85, 2.27951 }, Point { 2329.96, 2809.76 }), make_tuple(
          Frenet { 2898.11, 2.25583 }, Point { 2329.16, 2812.93 }), make_tuple(
          Frenet { 2901.53, 2.18491 }, Point { 2328.27, 2816.23 }), make_tuple(
          Frenet { 2904.77, 2.07342 }, Point { 2327.39, 2819.34 }), make_tuple(
          Frenet { 2908.34, 2.02104 }, Point { 2326.34, 2822.94 }), make_tuple(
          Frenet { 2912.68, 2.17181 }, Point { 2325.08, 2827.1 }), make_tuple(
          Frenet { 2916.81, 2.26174 }, Point { 2323.83, 2831.04 }), make_tuple(
          Frenet { 2920.54, 2.29964 }, Point { 2322.66, 2834.58 }), make_tuple(
          Frenet { 2924.68, 2.287 }, Point { 2321.32, 2838.49 }), make_tuple(
          Frenet { 2928.82, 2.20211 }, Point { 2319.9, 2842.38 }), make_tuple(
          Frenet { 2932.78, 2.0496 }, Point { 2318.47, 2846.08 }), make_tuple(
          Frenet { 2936.74, 2.12725 }, Point { 2316.92, 2849.94 }), make_tuple(
          Frenet { 2940.31, 2.26171 }, Point { 2315.55, 2853.24 }), make_tuple(
          Frenet { 2943.38, 2.34965 }, Point { 2314.35, 2856.07 }), make_tuple(
          Frenet { 2946.62, 2.40541 }, Point { 2313.05, 2859.04 }), make_tuple(
          Frenet { 2949.68, 2.41947 }, Point { 2311.79, 2861.83 }), make_tuple(
          Frenet { 2952.56, 2.40602 }, Point { 2310.58, 2864.44 }), make_tuple(
          Frenet { 2955.55, 2.35881 }, Point { 2309.29, 2867.14 }), make_tuple(
          Frenet { 2958.09, 2.29934 }, Point { 2308.18, 2869.42 }), make_tuple(
          Frenet { 2960.8, 2.20458 }, Point { 2306.96, 2871.85 }), make_tuple(
          Frenet { 2963.7, 2.06447 }, Point { 2305.63, 2874.43 }), make_tuple(
          Frenet { 2966.54, 2.15781 }, Point { 2304.18, 2877.13 }), make_tuple(
          Frenet { 2969.78, 2.26756 }, Point { 2302.61, 2879.96 }), make_tuple(
          Frenet { 2972.85, 2.33749 }, Point { 2301.09, 2882.64 }), make_tuple(
          Frenet { 2976.78, 2.381 }, Point { 2299.1, 2886.03 }), make_tuple(
          Frenet { 2980.56, 2.38214 }, Point { 2297.16, 2889.27 }), make_tuple(
          Frenet { 2984.12, 2.3543 }, Point { 2295.3, 2892.31 }), make_tuple(
          Frenet { 2988.23, 2.27995 }, Point { 2293.12, 2895.79 }), make_tuple(
          Frenet { 2992.19, 2.17843 }, Point { 2291, 2899.14 }), make_tuple(
          Frenet { 2995.95, 2.05685 }, Point { 2288.96, 2902.3 }), make_tuple(
          Frenet { 2999.39, 2.08282 }, Point { 2286.99, 2905.31 }), make_tuple(
          Frenet { 3003.13, 2.21848 }, Point { 2284.95, 2908.44 }), make_tuple(
          Frenet { 3006.34, 2.32282 }, Point { 2283.18, 2911.12 }), make_tuple(
          Frenet { 3009.37, 2.40555 }, Point { 2281.49, 2913.64 }), make_tuple(
          Frenet { 3012.23, 2.46348 }, Point { 2279.89, 2916.01 }), make_tuple(
          Frenet { 3014.91, 2.48907 }, Point { 2278.36, 2918.22 }), make_tuple(
          Frenet { 3017.43, 2.48665 }, Point { 2276.91, 2920.28 }), make_tuple(
          Frenet { 3019.78, 2.46593 }, Point { 2275.54, 2922.18 }), make_tuple(
          Frenet { 3021.72, 2.42824 }, Point { 2274.38, 2923.74 }), make_tuple(
          Frenet { 3024.05, 2.36524 }, Point { 2272.99, 2925.6 }), make_tuple(
          Frenet { 3026.8, 2.25481 }, Point { 2271.31, 2927.79 }), make_tuple(
          Frenet { 3029.5, 2.10701 }, Point { 2269.63, 2929.91 }), make_tuple(
          Frenet { 3032.06, 2.02549 }, Point { 2267.8, 2932.14 }), make_tuple(
          Frenet { 3034.82, 2.22078 }, Point { 2265.99, 2934.24 }), make_tuple(
          Frenet { 3038.08, 2.37118 }, Point { 2263.8, 2936.66 }), make_tuple(
          Frenet { 3041.86, 2.45131 }, Point { 2261.2, 2939.4 }), make_tuple(
          Frenet { 3045.11, 2.45774 }, Point { 2258.91, 2941.72 }), make_tuple(
          Frenet { 3048.88, 2.39721 }, Point { 2256.22, 2944.35 }), make_tuple(
          Frenet { 3052.83, 2.28134 }, Point { 2253.36, 2947.07 }), make_tuple(
          Frenet { 3056.95, 2.11594 }, Point { 2250.34, 2949.89 }), make_tuple(
          Frenet { 3061.29, 2.04794 }, Point { 2247.02, 2952.95 }), make_tuple(
          Frenet { 3065.2, 2.23362 }, Point { 2244.15, 2955.61 }), make_tuple(
          Frenet { 3068.95, 2.40266 }, Point { 2241.39, 2958.16 }), make_tuple(
          Frenet { 3072.54, 2.53913 }, Point { 2238.74, 2960.58 }), make_tuple(
          Frenet { 3075.95, 2.63079 }, Point { 2236.19, 2962.86 }), make_tuple(
          Frenet { 3078.89, 2.66267 }, Point { 2233.97, 2964.78 }), make_tuple(
          Frenet { 3082.29, 2.63511 }, Point { 2231.35, 2966.95 }), make_tuple(
          Frenet { 3085.19, 2.54268 }, Point { 2229.07, 2968.75 }), make_tuple(
          Frenet { 3088.16, 2.37128 }, Point { 2226.7, 2970.54 }), make_tuple(
          Frenet { 3090.91, 2.13409 }, Point { 2224.44, 2972.13 }), make_tuple(
          Frenet { 3092.85, 2.03743 }, Point { 2222.44, 2973.46 }), make_tuple(
          Frenet { 3095.4, 2.28626 }, Point { 2220.29, 2974.84 }), make_tuple(
          Frenet { 3097.84, 2.49346 }, Point { 2218.21, 2976.14 }), make_tuple(
          Frenet { 3100.73, 2.70072 }, Point { 2215.74, 2977.65 }), make_tuple(
          Frenet { 3103.81, 2.86302 }, Point { 2213.07, 2979.2 }), make_tuple(
          Frenet { 3107.06, 2.97164 }, Point { 2210.22, 2980.78 }), make_tuple(
          Frenet { 3110.5, 3.01702 }, Point { 2207.18, 2982.39 }), make_tuple(
          Frenet { 3114.12, 2.98109 }, Point { 2203.94, 2984.01 }), make_tuple(
          Frenet { 3117.94, 2.85574 }, Point { 2200.49, 2985.63 }), make_tuple(
          Frenet { 3121.92, 2.61047 }, Point { 2196.83, 2987.23 }), make_tuple(
          Frenet { 3125.65, 2.278 }, Point { 2193.36, 2988.64 }), make_tuple(
          Frenet { 3129.12, 2.13594 }, Point { 2189.63, 2990.03 }), make_tuple(
          Frenet { 3132.92, 2.48923 }, Point { 2186.01, 2991.23 }), make_tuple(
          Frenet { 3136.92, 2.72716 }, Point { 2182.17, 2992.36 }), make_tuple(
          Frenet { 3140.02, 2.83057 }, Point { 2179.17, 2993.16 }), make_tuple(
          Frenet { 3143.3, 2.86964 }, Point { 2175.99, 2993.95 }), make_tuple(
          Frenet { 3146.38, 2.85211 }, Point { 2172.98, 2994.62 }), make_tuple(
          Frenet { 3149.55, 2.7761 }, Point { 2169.87, 2995.27 }), make_tuple(
          Frenet { 3152, 2.68627 }, Point { 2167.47, 2995.73 }), make_tuple(
          Frenet { 3154.29, 2.581 }, Point { 2165.21, 2996.15 }), make_tuple(
          Frenet { 3156.42, 2.47008 }, Point { 2163.11, 2996.52 }), make_tuple(
          Frenet { 3158.65, 2.33872 }, Point { 2160.91, 2996.9 }), make_tuple(
          Frenet { 3161.28, 2.16667 }, Point { 2158.31, 2997.33 }), make_tuple(
          Frenet { 3163.96, 1.98285 }, Point { 2155.55, 2997.77 }), make_tuple(
          Frenet { 3166.78, 2.1865 }, Point { 2152.57, 2998.21 }), make_tuple(
          Frenet { 3169.99, 2.33723 }, Point { 2149.39, 2998.62 }), make_tuple(
          Frenet { 3173.39, 2.43463 }, Point { 2146, 2998.99 }), make_tuple(
          Frenet { 3177, 2.47098 }, Point { 2142.41, 2999.32 }), make_tuple(
          Frenet { 3180.78, 2.45317 }, Point { 2138.64, 2999.6 }), make_tuple(
          Frenet { 3185.15, 2.37488 }, Point { 2134.28, 2999.87 }), make_tuple(
          Frenet { 3189.28, 2.26295 }, Point { 2130.15, 3000.09 }), make_tuple(
          Frenet { 3192.98, 2.14569 }, Point { 2126.45, 3000.27 }), make_tuple(
          Frenet { 3197.09, 2.01441 }, Point { 2122.35, 3000.47 }), make_tuple(
          Frenet { 3200.91, 2.10137 }, Point { 2118.39, 3000.63 }), make_tuple(
          Frenet { 3205.06, 2.15816 }, Point { 2114.23, 3000.72 }), make_tuple(
          Frenet { 3208.65, 2.15995 }, Point { 2110.64, 3000.76 }), make_tuple(
          Frenet { 3212.04, 2.13549 }, Point { 2107.26, 3000.76 }), make_tuple(
          Frenet { 3215.24, 2.09804 }, Point { 2104.06, 3000.76 }), make_tuple(
          Frenet { 3218.25, 2.05493 }, Point { 2101.04, 3000.74 }), make_tuple(
          Frenet { 3220.82, 2.02903 }, Point { 2098.48, 3000.74 }), make_tuple(
          Frenet { 3223.5, 2.01183 }, Point { 2095.79, 3000.74 }), make_tuple(
          Frenet { 3225.77, 2.01167 }, Point { 2093.53, 3000.77 }), make_tuple(
          Frenet { 3228.36, 2.00452 }, Point { 2090.95, 3000.8 }), make_tuple(
          Frenet { 3230.52, 2.00051 }, Point { 2088.79, 3000.83 }), make_tuple(
          Frenet { 3233.07, 1.99801 }, Point { 2086.24, 3000.86 }), make_tuple(
          Frenet { 3235.32, 1.99366 }, Point { 2083.99, 3000.89 }), make_tuple(
          Frenet { 3238.01, 1.98809 }, Point { 2081.29, 3000.92 }), make_tuple(
          Frenet { 3240.87, 1.98897 }, Point { 2078.44, 3000.97 }), make_tuple(
          Frenet { 3244.22, 1.98637 }, Point { 2075.09, 3001.01 }), make_tuple(
          Frenet { 3247.11, 1.98477 }, Point { 2072.2, 3001.05 }), make_tuple(
          Frenet { 3250.49, 1.98117 }, Point { 2068.82, 3001.1 }), make_tuple(
          Frenet { 3254.05, 1.99351 }, Point { 2065.26, 3001.17 }), make_tuple(
          Frenet { 3257.81, 2.00313 }, Point { 2061.52, 3001.24 }), make_tuple(
          Frenet { 3261.33, 2.01954 }, Point { 2058, 3001.34 }), make_tuple(
          Frenet { 3265.39, 2.06436 }, Point { 2053.94, 3001.49 }), make_tuple(
          Frenet { 3269.91, 2.12329 }, Point { 2049.42, 3001.65 }), make_tuple(
          Frenet { 3274.44, 2.16875 }, Point { 2044.89, 3001.8 }), make_tuple(
          Frenet { 3278.57, 2.17591 }, Point { 2040.77, 3001.91 }), make_tuple(
          Frenet { 3282.67, 2.13298 }, Point { 2036.67, 3001.96 }), make_tuple(
          Frenet { 3286.59, 2.02672 }, Point { 2032.74, 3001.95 }), make_tuple(
          Frenet { 3290.2, 2.11048 }, Point { 2028.99, 3001.88 }), make_tuple(
          Frenet { 3293.78, 2.17978 }, Point { 2025.41, 3001.77 }), make_tuple(
          Frenet { 3296.85, 2.20917 }, Point { 2022.34, 3001.65 }), make_tuple(
          Frenet { 3300.09, 2.22656 }, Point { 2019.1, 3001.51 }), make_tuple(
          Frenet { 3303.75, 2.22587 }, Point { 2015.45, 3001.32 }), make_tuple(
          Frenet { 3306.86, 2.20783 }, Point { 2012.34, 3001.15 }), make_tuple(
          Frenet { 3309.76, 2.18083 }, Point { 2009.45, 3000.98 }), make_tuple(
          Frenet { 3311.96, 2.15565 }, Point { 2007.25, 3000.85 }), make_tuple(
          Frenet { 3314.23, 2.1247 }, Point { 2004.98, 3000.71 }), make_tuple(
          Frenet { 3316.63, 2.08695 }, Point { 2002.59, 3000.55 }), make_tuple(
          Frenet { 3318.97, 2.04737 }, Point { 2000.26, 3000.39 }), make_tuple(
          Frenet { 3321.47, 1.99862 }, Point { 1997.76, 3000.22 }), make_tuple(
          Frenet { 3324.11, 2.01726 }, Point { 1995.06, 3000.03 }), make_tuple(
          Frenet { 3327.3, 2.02891 }, Point { 1991.88, 2999.78 }), make_tuple(
          Frenet { 3330.69, 2.02143 }, Point { 1988.51, 2999.5 }), make_tuple(
          Frenet { 3333.61, 2.00348 }, Point { 1985.6, 2999.24 }), make_tuple(
          Frenet { 3337.02, 1.97818 }, Point { 1982.2, 2998.94 }), make_tuple(
          Frenet { 3340.98, 1.94271 }, Point { 1978.25, 2998.58 }), make_tuple(
          Frenet { 3344.35, 1.92072 }, Point { 1974.9, 2998.28 }), make_tuple(
          Frenet { 3348.27, 1.90842 }, Point { 1970.99, 2997.95 }), make_tuple(
          Frenet { 3351.93, 1.92162 }, Point { 1967.34, 2997.66 }), make_tuple(
          Frenet { 3356.09, 1.97219 }, Point { 1963.27, 2997.39 }), make_tuple(
          Frenet { 3359.76, 1.92892 }, Point { 1959.6, 2997.19 }), make_tuple(
          Frenet { 3363.72, 1.92772 }, Point { 1955.64, 2997.01 }), make_tuple(
          Frenet { 3367.5, 1.9491 }, Point { 1951.87, 2996.87 }), make_tuple(
          Frenet { 3370.74, 1.98318 }, Point { 1948.63, 2996.76 }), make_tuple(
          Frenet { 3374.17, 2.0202 }, Point { 1945.2, 2996.64 }), make_tuple(
          Frenet { 3377.43, 2.05443 }, Point { 1941.95, 2996.53 }), make_tuple(
          Frenet { 3380.18, 2.07756 }, Point { 1939.19, 2996.43 }), make_tuple(
          Frenet { 3383.09, 2.08643 }, Point { 1936.29, 2996.31 }), make_tuple(
          Frenet { 3386.33, 2.07241 }, Point { 1933.05, 2996.15 }), make_tuple(
          Frenet { 3388.87, 2.0406 }, Point { 1930.52, 2996.01 }), make_tuple(
          Frenet { 3391.76, 2.04207 }, Point { 1927.53, 2995.81 }), make_tuple(
          Frenet { 3394.68, 2.10609 }, Point { 1924.62, 2995.6 }), make_tuple(
          Frenet { 3397.78, 2.1548 }, Point { 1921.53, 2995.36 }), make_tuple(
          Frenet { 3400.68, 2.18521 }, Point { 1918.64, 2995.12 }), make_tuple(
          Frenet { 3404.45, 2.20784 }, Point { 1914.89, 2994.79 }), make_tuple(
          Frenet { 3407.67, 2.20524 }, Point { 1911.68, 2994.49 }), make_tuple(
          Frenet { 3411.04, 2.18301 }, Point { 1908.32, 2994.15 }), make_tuple(
          Frenet { 3414.95, 2.13561 }, Point { 1904.44, 2993.74 }), make_tuple(
          Frenet { 3419.03, 2.06081 }, Point { 1900.38, 2993.28 }), make_tuple(
          Frenet { 3423.46, 2.05005 }, Point { 1895.88, 2992.75 }), make_tuple(
          Frenet { 3427.41, 2.12535 }, Point { 1891.96, 2992.26 }), make_tuple(
          Frenet { 3431.16, 2.17467 }, Point { 1888.24, 2991.77 }), make_tuple(
          Frenet { 3434.72, 2.19898 }, Point { 1884.71, 2991.28 }), make_tuple(
          Frenet { 3438.11, 2.20255 }, Point { 1881.37, 2990.8 }), make_tuple(
          Frenet { 3441.33, 2.19157 }, Point { 1878.18, 2990.33 }), make_tuple(
          Frenet { 3444.07, 2.17047 }, Point { 1875.47, 2989.91 }), make_tuple(
          Frenet { 3446.95, 2.14252 }, Point { 1872.62, 2989.47 }), make_tuple(
          Frenet { 3449.39, 2.11347 }, Point { 1870.21, 2989.09 }), make_tuple(
          Frenet { 3451.68, 2.08057 }, Point { 1867.95, 2988.73 }), make_tuple(
          Frenet { 3454.29, 2.0268 }, Point { 1865.38, 2988.3 }), make_tuple(
          Frenet { 3456.93, 2.03536 }, Point { 1862.68, 2987.84 }), make_tuple(
          Frenet { 3459.91, 2.0536 }, Point { 1859.75, 2987.3 }), make_tuple(
          Frenet { 3462.52, 2.04343 }, Point { 1857.19, 2986.81 }), make_tuple(
          Frenet { 3465.6, 2.01774 }, Point { 1854.17, 2986.21 }), make_tuple(
          Frenet { 3468.84, 1.97964 }, Point { 1850.99, 2985.57 }), make_tuple(
          Frenet { 3471.9, 1.9393 }, Point { 1848, 2984.96 }), make_tuple(
          Frenet { 3475.45, 1.89291 }, Point { 1844.52, 2984.25 }), make_tuple(
          Frenet { 3479.91, 1.85825 }, Point { 1840.14, 2983.38 }), make_tuple(
          Frenet { 3483.42, 1.86506 }, Point { 1836.69, 2982.74 }), make_tuple(
          Frenet { 3487.9, 1.93734 }, Point { 1832.28, 2981.97 }), make_tuple(
          Frenet { 3492.08, 1.94795 }, Point { 1828.29, 2981.36 }), make_tuple(
          Frenet { 3495.95, 1.84668 }, Point { 1824.45, 2980.81 }), make_tuple(
          Frenet { 3500.04, 1.77593 }, Point { 1820.4, 2980.26 }), make_tuple(
          Frenet { 3503.56, 1.74162 }, Point { 1816.91, 2979.82 }), make_tuple(
          Frenet { 3506.92, 1.73298 }, Point { 1813.57, 2979.42 }), make_tuple(
          Frenet { 3509.8, 1.74408 }, Point { 1810.7, 2979.1 }), make_tuple(
          Frenet { 3513.41, 1.77724 }, Point { 1807.12, 2978.71 }), make_tuple(
          Frenet { 3516.51, 1.81779 }, Point { 1804.04, 2978.39 }), make_tuple(
          Frenet { 3519.39, 1.86451 }, Point { 1801.16, 2978.1 }), make_tuple(
          Frenet { 3522.07, 1.91352 }, Point { 1798.5, 2977.84 }), make_tuple(
          Frenet { 3524.48, 1.95347 }, Point { 1796.1, 2977.6 }), make_tuple(
          Frenet { 3527.3, 2.00115 }, Point { 1793.3, 2977.32 }), make_tuple(
          Frenet { 3530.09, 1.98508 }, Point { 1790.59, 2977.05 }), make_tuple(
          Frenet { 3532.66, 1.96059 }, Point { 1788.02, 2976.81 }), make_tuple(
          Frenet { 3535.69, 1.95538 }, Point { 1785.01, 2976.56 }), make_tuple(
          Frenet { 3539.23, 1.9649 }, Point { 1781.48, 2976.27 }), make_tuple(
          Frenet { 3542.62, 1.98671 }, Point { 1778.1, 2976.01 }), make_tuple(
          Frenet { 3546.18, 2.02061 }, Point { 1774.54, 2975.75 }), make_tuple(
          Frenet { 3550.34, 2.06173 }, Point { 1770.39, 2975.45 }), make_tuple(
          Frenet { 3553.9, 2.09277 }, Point { 1766.85, 2975.18 }), make_tuple(
          Frenet { 3558.41, 2.10848 }, Point { 1762.35, 2974.83 }), make_tuple(
          Frenet { 3561.96, 2.10455 }, Point { 1758.82, 2974.53 }), make_tuple(
          Frenet { 3565.72, 2.07545 }, Point { 1755.07, 2974.19 }), make_tuple(
          Frenet { 3569.23, 2.05466 }, Point { 1751.5, 2973.84 }), make_tuple(
          Frenet { 3572.97, 2.12242 }, Point { 1747.78, 2973.45 }), make_tuple(
          Frenet { 3575.86, 2.15944 }, Point { 1744.9, 2973.14 }), make_tuple(
          Frenet { 3579.21, 2.18547 }, Point { 1741.58, 2972.76 }), make_tuple(
          Frenet { 3581.79, 2.19639 }, Point { 1739.02, 2972.46 }), make_tuple(
          Frenet { 3584.47, 2.19915 }, Point { 1736.35, 2972.14 }), make_tuple(
          Frenet { 3587.23, 2.19053 }, Point { 1733.62, 2971.79 }), make_tuple(
          Frenet { 3589.57, 2.1822 }, Point { 1731.29, 2971.5 }), make_tuple(
          Frenet { 3591.83, 2.17004 }, Point { 1729.05, 2971.22 }), make_tuple(
          Frenet { 3594.49, 2.14845 }, Point { 1726.41, 2970.87 }), make_tuple(
          Frenet { 3597.36, 2.12454 }, Point { 1723.57, 2970.5 }), make_tuple(
          Frenet { 3600.7, 2.0781 }, Point { 1720.25, 2970.05 }), make_tuple(
          Frenet { 3603.93, 2.01729 }, Point { 1717.07, 2969.6 }), make_tuple(
          Frenet { 3607.62, 2.0404 }, Point { 1713.32, 2969.05 }), make_tuple(
          Frenet { 3611.24, 2.06965 }, Point { 1709.75, 2968.49 }), make_tuple(
          Frenet { 3615.04, 2.08355 }, Point { 1706, 2967.89 }), make_tuple(
          Frenet { 3618.99, 2.07772 }, Point { 1702.1, 2967.25 }), make_tuple(
          Frenet { 3623.1, 2.05499 }, Point { 1698.04, 2966.56 }), make_tuple(
          Frenet { 3627.21, 2.02582 }, Point { 1693.99, 2965.87 }), make_tuple(
          Frenet { 3631.14, 2.00004 }, Point { 1690.12, 2965.21 }), make_tuple(
          Frenet { 3634.89, 1.98644 }, Point { 1686.43, 2964.59 }), make_tuple(
          Frenet { 3638.09, 1.9773 }, Point { 1683.26, 2964.07 }), make_tuple(
          Frenet { 3641.83, 1.97983 }, Point { 1679.57, 2963.47 }), make_tuple(
          Frenet { 3645.37, 1.97177 }, Point { 1676.1, 2962.91 }), make_tuple(
          Frenet { 3648.68, 1.949 }, Point { 1672.83, 2962.39 }), make_tuple(
          Frenet { 3651.5, 1.92527 }, Point { 1670.05, 2961.93 }), make_tuple(
          Frenet { 3654.12, 1.90437 }, Point { 1667.46, 2961.51 }), make_tuple(
          Frenet { 3656.82, 1.88406 }, Point { 1664.8, 2961.08 }), make_tuple(
          Frenet { 3658.92, 1.87063 }, Point { 1662.72, 2960.74 }), make_tuple(
          Frenet { 3661.69, 1.85507 }, Point { 1659.99, 2960.3 }), make_tuple(
          Frenet { 3664.38, 1.84235 }, Point { 1657.34, 2959.88 }), make_tuple(
          Frenet { 3667.24, 1.83956 }, Point { 1654.5, 2959.44 }), make_tuple(
          Frenet { 3670.27, 1.85745 }, Point { 1651.51, 2958.99 }), make_tuple(
          Frenet { 3673.82, 1.89746 }, Point { 1647.99, 2958.49 }), make_tuple(
          Frenet { 3677.22, 1.95986 }, Point { 1644.63, 2958.03 }), make_tuple(
          Frenet { 3680.5, 1.92605 }, Point { 1641.45, 2957.62 }), make_tuple(
          Frenet { 3684.24, 1.88771 }, Point { 1637.75, 2957.18 }), make_tuple(
          Frenet { 3688.15, 1.87021 }, Point { 1633.86, 2956.73 }), make_tuple(
          Frenet { 3693.07, 1.88346 }, Point { 1628.97, 2956.21 }), make_tuple(
          Frenet { 3697.17, 1.90967 }, Point { 1624.89, 2955.79 }), make_tuple(
          Frenet { 3701.24, 1.94426 }, Point { 1620.84, 2955.38 }), make_tuple(
          Frenet { 3705.13, 1.97975 }, Point { 1616.97, 2954.99 }), make_tuple(
          Frenet { 3709.56, 2.01105 }, Point { 1612.56, 2954.53 }), make_tuple(
          Frenet { 3713.44, 2.03324 }, Point { 1608.7, 2954.13 }), make_tuple(
          Frenet { 3717.11, 2.0381 }, Point { 1605.05, 2953.74 }), make_tuple(
          Frenet { 3720.27, 2.0242 }, Point { 1601.92, 2953.38 }), make_tuple(
          Frenet { 3722.95, 1.99869 }, Point { 1599.25, 2953.06 }), make_tuple(
          Frenet { 3725.76, 2.01005 }, Point { 1596.42, 2952.71 }), make_tuple(
          Frenet { 3728.44, 2.02068 }, Point { 1593.75, 2952.36 }), make_tuple(
          Frenet { 3730.95, 2.021 }, Point { 1591.27, 2952.03 }), make_tuple(
          Frenet { 3733.3, 2.01071 }, Point { 1588.94, 2951.7 }), make_tuple(
          Frenet { 3736.06, 1.98219 }, Point { 1586.21, 2951.31 }), make_tuple(
          Frenet { 3738.47, 1.94514 }, Point { 1583.82, 2950.95 }), make_tuple(
          Frenet { 3741.95, 1.8873 }, Point { 1580.39, 2950.43 }), make_tuple(
          Frenet { 3745.34, 1.82771 }, Point { 1577.03, 2949.92 }), make_tuple(
          Frenet { 3748.27, 1.77296 }, Point { 1574.13, 2949.47 }), make_tuple(
          Frenet { 3752.04, 1.70579 }, Point { 1570.41, 2948.9 }), make_tuple(
          Frenet { 3755.66, 1.64921 }, Point { 1566.83, 2948.36 }), make_tuple(
          Frenet { 3759.03, 1.60696 }, Point { 1563.49, 2947.87 }), make_tuple(
          Frenet { 3762.95, 1.57509 }, Point { 1559.61, 2947.32 }), make_tuple(
          Frenet { 3767.43, 1.56963 }, Point { 1555.17, 2946.72 }), make_tuple(
          Frenet { 3771.11, 1.59421 }, Point { 1551.52, 2946.25 }), make_tuple(
          Frenet { 3775.2, 1.64988 }, Point { 1547.47, 2945.76 }), make_tuple(
          Frenet { 3780.05, 1.78446 }, Point { 1542.64, 2945.25 }), make_tuple(
          Frenet { 3783.53, 1.93135 }, Point { 1539.17, 2944.93 }), make_tuple(
          Frenet { 3787.46, 1.83842 }, Point { 1535.46, 2944.63 }), make_tuple(
          Frenet { 3791.32, 1.69801 }, Point { 1531.6, 2944.38 }), make_tuple(
          Frenet { 3794.65, 1.61319 }, Point { 1528.27, 2944.2 }), make_tuple(
          Frenet { 3798.15, 1.55011 }, Point { 1524.78, 2944.03 }), make_tuple(
          Frenet { 3800.85, 1.51633 }, Point { 1522.08, 2943.91 }), make_tuple(
          Frenet { 3804.21, 1.49927 }, Point { 1518.72, 2943.8 }), make_tuple(
          Frenet { 3807.09, 1.50586 }, Point { 1515.84, 2943.72 }), make_tuple(
          Frenet { 3809.76, 1.52618 }, Point { 1513.17, 2943.66 }), make_tuple(
          Frenet { 3812.03, 1.55244 }, Point { 1510.91, 2943.62 }), make_tuple(
          Frenet { 3814.24, 1.5803 }, Point { 1508.7, 2943.58 }), make_tuple(
          Frenet { 3816.84, 1.61352 }, Point { 1506.09, 2943.54 }), make_tuple(
          Frenet { 3819.38, 1.65003 }, Point { 1503.56, 2943.5 }), make_tuple(
          Frenet { 3822.36, 1.69448 }, Point { 1500.58, 2943.45 }), make_tuple(
          Frenet { 3824.95, 1.73561 }, Point { 1497.98, 2943.42 }), make_tuple(
          Frenet { 3828.33, 1.79107 }, Point { 1494.61, 2943.37 }), make_tuple(
          Frenet { 3831.22, 1.8391 }, Point { 1491.71, 2943.33 }), make_tuple(
          Frenet { 3834.62, 1.89519 }, Point { 1488.31, 2943.29 }), make_tuple(
          Frenet { 3838.2, 1.94443 }, Point { 1484.73, 2943.23 }), make_tuple(
          Frenet { 3841.56, 1.98239 }, Point { 1481.37, 2943.17 }), make_tuple(
          Frenet { 3845.51, 1.95952 }, Point { 1477.46, 2943.1 }), make_tuple(
          Frenet { 3849.99, 1.93127 }, Point { 1472.99, 2943.02 }), make_tuple(
          Frenet { 3854.09, 1.90855 }, Point { 1468.88, 2942.95 }), make_tuple(
          Frenet { 3858.6, 1.88655 }, Point { 1464.37, 2942.88 }), make_tuple(
          Frenet { 3862.3, 1.87726 }, Point { 1460.68, 2942.82 }), make_tuple(
          Frenet { 3866.38, 1.87435 }, Point { 1456.59, 2942.78 }), make_tuple(
          Frenet { 3870.31, 1.87379 }, Point { 1452.67, 2942.73 }), make_tuple(
          Frenet { 3874.05, 1.87315 }, Point { 1448.92, 2942.69 }), make_tuple(
          Frenet { 3877.62, 1.87952 }, Point { 1445.36, 2942.65 }), make_tuple(
          Frenet { 3881.35, 1.89357 }, Point { 1441.63, 2942.63 }), make_tuple(
          Frenet { 3884.84, 1.9172 }, Point { 1438.13, 2942.61 }), make_tuple(
          Frenet { 3888.12, 1.94525 }, Point { 1434.85, 2942.6 }), make_tuple(
          Frenet { 3890.92, 1.97273 }, Point { 1432.05, 2942.6 }), make_tuple(
          Frenet { 3893.55, 2.00334 }, Point { 1429.43, 2942.6 }), make_tuple(
          Frenet { 3895.83, 1.98583 }, Point { 1427.2, 2942.6 }), make_tuple(
          Frenet { 3897.68, 1.95969 }, Point { 1425.35, 2942.6 }), make_tuple(
          Frenet { 3900.14, 1.93339 }, Point { 1422.88, 2942.61 }), make_tuple(
          Frenet { 3903.03, 1.91158 }, Point { 1420, 2942.64 }), make_tuple(
          Frenet { 3906.13, 1.88949 }, Point { 1416.89, 2942.66 }), make_tuple(
          Frenet { 3909.14, 1.86798 }, Point { 1413.88, 2942.68 }), make_tuple(
          Frenet { 3912.05, 1.85593 }, Point { 1410.97, 2942.72 }), make_tuple(
          Frenet { 3915.46, 1.8512 }, Point { 1407.57, 2942.76 }), make_tuple(
          Frenet { 3918.66, 1.85459 }, Point { 1404.36, 2942.82 }), make_tuple(
          Frenet { 3922.39, 1.86526 }, Point { 1400.64, 2942.88 }), make_tuple(
          Frenet { 3926.28, 1.88327 }, Point { 1396.75, 2942.96 }), make_tuple(
          Frenet { 3929.93, 1.91384 }, Point { 1393.1, 2943.05 }), make_tuple(
          Frenet { 3934.03, 1.96377 }, Point { 1389, 2943.16 }), make_tuple(
          Frenet { 3938.19, 2.01132 }, Point { 1384.9, 2943.28 }), make_tuple(
          Frenet { 3941.74, 1.96459 }, Point { 1381.35, 2943.38 }), make_tuple(
          Frenet { 3945.53, 1.90303 }, Point { 1377.56, 2943.48 }), make_tuple(
          Frenet { 3949.14, 1.84804 }, Point { 1373.95, 2943.58 }), make_tuple(
          Frenet { 3952.21, 1.80569 }, Point { 1370.88, 2943.66 }), make_tuple(
          Frenet { 3955.46, 1.76473 }, Point { 1367.63, 2943.76 }), make_tuple(
          Frenet { 3959.12, 1.71662 }, Point { 1363.97, 2943.86 }), make_tuple(
          Frenet { 3961.96, 1.68382 }, Point { 1361.14, 2943.95 }), make_tuple(
          Frenet { 3964.61, 1.65758 }, Point { 1358.48, 2944.03 }), make_tuple(
          Frenet { 3967.07, 1.64059 }, Point { 1356.02, 2944.12 }), make_tuple(
          Frenet { 3969.57, 1.62918 }, Point { 1353.53, 2944.21 }), make_tuple(
          Frenet { 3971.5, 1.62481 }, Point { 1351.6, 2944.29 }), make_tuple(
          Frenet { 3973.81, 1.6258 }, Point { 1349.29, 2944.39 }), make_tuple(
          Frenet { 3976.58, 1.63139 }, Point { 1346.53, 2944.51 }), make_tuple(
          Frenet { 3979.26, 1.6417 }, Point { 1343.84, 2944.63 }), make_tuple(
          Frenet { 3982.42, 1.66814 }, Point { 1340.69, 2944.79 }), make_tuple(
          Frenet { 3985.48, 1.70395 }, Point { 1337.64, 2944.95 }), make_tuple(
          Frenet { 3989.05, 1.7547 }, Point { 1334.07, 2945.15 }), make_tuple(
          Frenet { 3992.11, 1.81166 }, Point { 1331.02, 2945.34 }), make_tuple(
          Frenet { 3995.68, 1.90042 }, Point { 1327.45, 2945.57 }), make_tuple(
          Frenet { 3999.41, 2.02461 }, Point { 1323.73, 2945.86 }), make_tuple(
          Frenet { 4003.88, 1.87704 }, Point { 1319.45, 2946.23 }), make_tuple(
          Frenet { 4007.95, 1.78428 }, Point { 1315.4, 2946.66 }), make_tuple(
          Frenet { 4011.62, 1.75117 }, Point { 1311.76, 2947.1 }), make_tuple(
          Frenet { 4016.1, 1.76141 }, Point { 1307.32, 2947.69 }), make_tuple(
          Frenet { 4019.62, 1.79778 }, Point { 1303.83, 2948.18 }), make_tuple(
          Frenet { 4023.36, 1.85207 }, Point { 1300.13, 2948.72 }), make_tuple(
          Frenet { 4026.95, 1.92055 }, Point { 1296.58, 2949.25 }), make_tuple(
          Frenet { 4030.37, 1.9861 }, Point { 1293.2, 2949.75 }), make_tuple(
          Frenet { 4033.59, 2.04151 }, Point { 1290.01, 2950.22 }), make_tuple(
          Frenet { 4036.94, 2.08128 }, Point { 1286.69, 2950.69 }), make_tuple(
          Frenet { 4039.79, 2.0981 }, Point { 1283.87, 2951.08 }), make_tuple(
          Frenet { 4042.21, 2.09907 }, Point { 1281.47, 2951.39 }), make_tuple(
          Frenet { 4044.73, 2.09137 }, Point { 1278.97, 2951.7 }), make_tuple(
          Frenet { 4047.42, 2.05363 }, Point { 1276.29, 2952.01 }), make_tuple(
          Frenet { 4050.29, 1.9861 }, Point { 1273.43, 2952.32 }), make_tuple(
          Frenet { 4053.2, 2.07627 }, Point { 1270.43, 2952.62 }), make_tuple(
          Frenet { 4056.4, 2.16275 }, Point { 1267.24, 2952.93 }), make_tuple(
          Frenet { 4059.8, 2.24299 }, Point { 1263.86, 2953.25 }), make_tuple(
          Frenet { 4063.38, 2.31784 }, Point { 1260.29, 2953.58 }), make_tuple(
          Frenet { 4067.54, 2.38287 }, Point { 1256.15, 2953.94 }), make_tuple(
          Frenet { 4072.32, 2.43072 }, Point { 1251.38, 2954.32 }), make_tuple(
          Frenet { 4076.44, 2.44015 }, Point { 1247.28, 2954.62 }), make_tuple(
          Frenet { 4081.29, 2.41545 }, Point { 1242.43, 2954.94 }), make_tuple(
          Frenet { 4085.14, 2.37484 }, Point { 1238.59, 2955.17 }), make_tuple(
          Frenet { 4088.83, 2.31336 }, Point { 1234.9, 2955.37 }), make_tuple(
          Frenet { 4092.04, 2.24077 }, Point { 1231.7, 2955.53 }), make_tuple(
          Frenet { 4095.1, 2.15756 }, Point { 1228.64, 2955.66 }), make_tuple(
          Frenet { 4098, 2.06281 }, Point { 1225.74, 2955.77 }), make_tuple(
          Frenet { 4101.56, 2.07302 }, Point { 1222.03, 2955.88 }), make_tuple(
          Frenet { 4104.73, 2.15295 }, Point { 1218.87, 2955.96 }), make_tuple(
          Frenet { 4107.4, 2.21354 }, Point { 1216.2, 2956.02 }), make_tuple(
          Frenet { 4109.86, 2.25572 }, Point { 1213.73, 2956.06 }), make_tuple(
          Frenet { 4112.14, 2.28924 }, Point { 1211.46, 2956.09 }), make_tuple(
          Frenet { 4114.3, 2.31439 }, Point { 1209.29, 2956.11 }), make_tuple(
          Frenet { 4116.4, 2.33243 }, Point { 1207.2, 2956.13 }), make_tuple(
          Frenet { 4118.88, 2.35288 }, Point { 1204.71, 2956.15 }), make_tuple(
          Frenet { 4121.55, 2.36246 }, Point { 1202.05, 2956.16 }), make_tuple(
          Frenet { 4124.11, 2.37099 }, Point { 1199.49, 2956.16 }), make_tuple(
          Frenet { 4127.11, 2.37172 }, Point { 1196.48, 2956.16 }), make_tuple(
          Frenet { 4129.98, 2.36236 }, Point { 1193.62, 2956.15 }), make_tuple(
          Frenet { 4133, 2.35089 }, Point { 1190.6, 2956.14 }), make_tuple(
          Frenet { 4136.88, 2.32722 }, Point { 1186.71, 2956.11 }), make_tuple(
          Frenet { 4141.37, 2.2868 }, Point { 1182.23, 2956.07 }), make_tuple(
          Frenet { 4144.91, 2.24585 }, Point { 1178.69, 2956.03 }), make_tuple(
          Frenet { 4149.02, 2.19008 }, Point { 1174.58, 2955.97 }), make_tuple(
          Frenet { 4153.88, 2.11691 }, Point { 1169.72, 2955.89 }), make_tuple(
          Frenet { 4157.38, 2.05884 }, Point { 1166.21, 2955.83 }), make_tuple(
          Frenet { 4160.65, 2.03552 }, Point { 1162.84, 2955.77 }), make_tuple(
          Frenet { 4164.59, 2.14984 }, Point { 1158.9, 2955.67 }), make_tuple(
          Frenet { 4167.97, 2.22338 }, Point { 1155.52, 2955.57 }), make_tuple(
          Frenet { 4171.2, 2.27255 }, Point { 1152.29, 2955.44 }), make_tuple(
          Frenet { 4174.27, 2.30634 }, Point { 1149.23, 2955.31 }), make_tuple(
          Frenet { 4176.85, 2.32718 }, Point { 1146.65, 2955.2 }), make_tuple(
          Frenet { 4179.54, 2.33867 }, Point { 1143.96, 2955.06 }), make_tuple(
          Frenet { 4182.59, 2.34249 }, Point { 1140.92, 2954.9 }), make_tuple(
          Frenet { 4185.58, 2.32236 }, Point { 1137.93, 2954.72 }), make_tuple(
          Frenet { 4188.49, 2.2872 }, Point { 1135.03, 2954.53 }), make_tuple(
          Frenet { 4191.59, 2.22367 }, Point { 1131.93, 2954.3 }), make_tuple(
          Frenet { 4194.87, 2.12272 }, Point { 1128.66, 2954.03 }), make_tuple(
          Frenet { 4197.79, 2.09542 }, Point { 1125.61, 2953.74 }), make_tuple(
          Frenet { 4201.36, 2.19441 }, Point { 1122.06, 2953.41 }), make_tuple(
          Frenet { 4204.72, 2.28342 }, Point { 1118.71, 2953.09 }), make_tuple(
          Frenet { 4208.62, 2.37549 }, Point { 1114.83, 2952.7 }), make_tuple(
          Frenet { 4212.7, 2.45777 }, Point { 1110.77, 2952.28 }), make_tuple(
          Frenet { 4216.82, 2.53377 }, Point { 1106.67, 2951.86 }), make_tuple(
          Frenet { 4220.78, 2.58543 }, Point { 1102.73, 2951.43 }), make_tuple(
          Frenet { 4224.93, 2.61565 }, Point { 1098.62, 2950.95 }), make_tuple(
          Frenet { 4228.5, 2.61721 }, Point { 1095.07, 2950.52 }), make_tuple(
          Frenet { 4231.89, 2.59465 }, Point { 1091.7, 2950.08 }), make_tuple(
          Frenet { 4235.41, 2.53606 }, Point { 1088.22, 2949.59 }), make_tuple(
          Frenet { 4239, 2.43586 }, Point { 1084.67, 2949.06 }), make_tuple(
          Frenet { 4242.36, 2.30312 }, Point { 1081.36, 2948.52 }), make_tuple(
          Frenet { 4244.96, 2.16716 }, Point { 1078.79, 2948.06 }), make_tuple(
          Frenet { 4247.38, 2.01713 }, Point { 1076.41, 2947.62 }), make_tuple(
          Frenet { 4249.8, 2.082 }, Point { 1073.8, 2947.11 }), make_tuple(
          Frenet { 4252.62, 2.1924 }, Point { 1071.03, 2946.55 }), make_tuple(
          Frenet { 4255.06, 2.27899 }, Point { 1068.64, 2946.07 }), make_tuple(
          Frenet { 4257.63, 2.36125 }, Point { 1066.12, 2945.54 }), make_tuple(
          Frenet { 4260.68, 2.44487 }, Point { 1063.14, 2944.91 }), make_tuple(
          Frenet { 4263.9, 2.51555 }, Point { 1059.99, 2944.22 }), make_tuple(
          Frenet { 4267.28, 2.56998 }, Point { 1056.69, 2943.47 }), make_tuple(
          Frenet { 4270.47, 2.6042 }, Point { 1053.59, 2942.76 }), make_tuple(
          Frenet { 4274.2, 2.62115 }, Point { 1049.96, 2941.9 }), make_tuple(
          Frenet { 4278.51, 2.61812 }, Point { 1045.77, 2940.88 }), make_tuple(
          Frenet { 4282.27, 2.59752 }, Point { 1042.12, 2939.98 }), make_tuple(
          Frenet { 4285.85, 2.56448 }, Point { 1038.64, 2939.1 }), make_tuple(
          Frenet { 4289.56, 2.52455 }, Point { 1035.05, 2938.19 }), make_tuple(
          Frenet { 4292.75, 2.47721 }, Point { 1031.97, 2937.4 }), make_tuple(
          Frenet { 4295.48, 2.42842 }, Point { 1029.32, 2936.71 }), make_tuple(
          Frenet { 4298.35, 2.37275 }, Point { 1026.55, 2935.98 }), make_tuple(
          Frenet { 4301.6, 2.31337 }, Point { 1023.4, 2935.16 }), make_tuple(
          Frenet { 4305.09, 2.24308 }, Point { 1020.03, 2934.27 }), make_tuple(
          Frenet { 4308.18, 2.17552 }, Point { 1017.04, 2933.48 }), make_tuple(
          Frenet { 4311.12, 2.10567 }, Point { 1014.19, 2932.72 }), make_tuple(
          Frenet { 4314.21, 2.03338 }, Point { 1011.21, 2931.92 }), make_tuple(
          Frenet { 4318.07, 2.08948 }, Point { 1007.38, 2930.89 }), make_tuple(
          Frenet { 4321.46, 2.14463 }, Point { 1004.11, 2929.96 }), make_tuple(
          Frenet { 4325.37, 2.16405 }, Point { 1000.36, 2928.86 }), make_tuple(
          Frenet { 4330.28, 2.13819 }, Point { 995.661, 2927.42 }), make_tuple(
          Frenet { 4334.76, 2.07882 }, Point { 991.39, 2926.08 }), make_tuple(
          Frenet { 4338.66, 2.01116 }, Point { 987.674, 2924.89 }), make_tuple(
          Frenet { 4342.36, 1.9397 }, Point { 984.151, 2923.76 }), make_tuple(
          Frenet { 4345.88, 1.87215 }, Point { 980.802, 2922.69 }), make_tuple(
          Frenet { 4348.93, 1.82094 }, Point { 977.895, 2921.76 }), make_tuple(
          Frenet { 4352.45, 1.77738 }, Point { 974.529, 2920.71 }), make_tuple(
          Frenet { 4355.48, 1.75647 }, Point { 971.638, 2919.82 }), make_tuple(
          Frenet { 4358.59, 1.75282 }, Point { 968.66, 2918.92 }), make_tuple(
          Frenet { 4360.99, 1.76047 }, Point { 966.362, 2918.24 }), make_tuple(
          Frenet { 4363.45, 1.7858 }, Point { 963.993, 2917.56 }), make_tuple(
          Frenet { 4365.76, 1.82913 }, Point { 961.775, 2916.94 }), make_tuple(
          Frenet { 4368.11, 1.89404 }, Point { 959.501, 2916.32 }), make_tuple(
          Frenet { 4370.18, 1.96546 }, Point { 957.499, 2915.8 }), make_tuple(
          Frenet { 4372.84, 1.89115 }, Point { 955.115, 2915.2 }), make_tuple(
          Frenet { 4375.23, 1.77187 }, Point { 952.791, 2914.64 }), make_tuple(
          Frenet { 4378.33, 1.64559 }, Point { 949.77, 2913.95 }), make_tuple(
          Frenet { 4381.34, 1.5583 }, Point { 946.824, 2913.3 }), make_tuple(
          Frenet { 4384.52, 1.49565 }, Point { 943.715, 2912.66 }), make_tuple(
          Frenet { 4387.89, 1.46396 }, Point { 940.404, 2912 }), make_tuple(
          Frenet { 4391.09, 1.47357 }, Point { 937.259, 2911.42 }), make_tuple(
          Frenet { 4395.2, 1.53069 }, Point { 933.213, 2910.72 }), make_tuple(
          Frenet { 4399.11, 1.62637 }, Point { 929.352, 2910.09 }), make_tuple(
          Frenet { 4403.6, 1.7849 }, Point { 924.905, 2909.42 }), make_tuple(
          Frenet { 4407.68, 1.97498 }, Point { 920.858, 2908.85 }), make_tuple(
          Frenet { 4411.91, 1.89048 }, Point { 916.843, 2908.32 }), make_tuple(
          Frenet { 4416.18, 1.76643 }, Point { 912.61, 2907.79 }), make_tuple(
          Frenet { 4419.88, 1.68495 }, Point { 908.936, 2907.36 }), make_tuple(
          Frenet { 4423.41, 1.63131 }, Point { 905.417, 2906.97 }), make_tuple(
          Frenet { 4426.78, 1.60426 }, Point { 902.071, 2906.62 }), make_tuple(
          Frenet { 4429.97, 1.59634 }, Point { 898.899, 2906.31 }), make_tuple(
          Frenet { 4432.99, 1.60044 }, Point { 895.884, 2906.03 }), make_tuple(
          Frenet { 4436.11, 1.62391 }, Point { 892.778, 2905.76 }), make_tuple(
          Frenet { 4439.27, 1.65306 }, Point { 889.635, 2905.48 }), make_tuple(
          Frenet { 4441.55, 1.67925 }, Point { 887.362, 2905.29 }), make_tuple(
          Frenet { 4444.26, 1.71516 }, Point { 884.662, 2905.07 }), make_tuple(
          Frenet { 4446.84, 1.75706 }, Point { 882.085, 2904.87 }), make_tuple(
          Frenet { 4449.87, 1.81849 }, Point { 879.063, 2904.64 }), make_tuple(
          Frenet { 4453.08, 1.89617 }, Point { 875.862, 2904.41 }), make_tuple(
          Frenet { 4456.87, 1.98794 }, Point { 872.181, 2904.16 }), make_tuple(
          Frenet { 4460.43, 1.91907 }, Point { 868.619, 2903.93 }), make_tuple(
          Frenet { 4464.2, 1.84944 }, Point { 864.862, 2903.69 }), make_tuple(
          Frenet { 4468.55, 1.77185 }, Point { 860.516, 2903.41 }), make_tuple(
          Frenet { 4472.66, 1.70358 }, Point { 856.42, 2903.16 }), make_tuple(
          Frenet { 4477.53, 1.63648 }, Point { 851.557, 2902.87 }), make_tuple(
          Frenet { 4481.4, 1.5937 }, Point { 847.692, 2902.65 }), make_tuple(
          Frenet { 4484.73, 1.56547 }, Point { 844.362, 2902.47 }), make_tuple(
          Frenet { 4488.62, 1.54322 }, Point { 840.48, 2902.26 }), make_tuple(
          Frenet { 4491.97, 1.53479 }, Point { 837.133, 2902.1 }), make_tuple(
          Frenet { 4495.18, 1.53762 }, Point { 833.929, 2901.96 }), make_tuple(
          Frenet { 4498.49, 1.55239 }, Point { 830.618, 2901.82 }), make_tuple(
          Frenet { 4500.77, 1.57084 }, Point { 828.34, 2901.74 }), make_tuple(
          Frenet { 4503.44, 1.60404 }, Point { 825.671, 2901.65 }), make_tuple(
          Frenet { 4505.95, 1.63792 }, Point { 823.163, 2901.57 }), make_tuple(
          Frenet { 4508.28, 1.68267 }, Point { 820.832, 2901.5 }), make_tuple(
          Frenet { 4510.75, 1.7361 }, Point { 818.36, 2901.44 }), make_tuple(
          Frenet { 4513.39, 1.79855 }, Point { 815.729, 2901.39 }), make_tuple(
          Frenet { 4516.53, 1.88456 }, Point { 812.583, 2901.33 }), make_tuple(
          Frenet { 4520.07, 1.97846 }, Point { 809.225, 2901.29 }), make_tuple(
          Frenet { 4523.62, 1.82066 }, Point { 805.671, 2901.3 }), make_tuple(
          Frenet { 4527.03, 1.71932 }, Point { 802.254, 2901.35 }), make_tuple(
          Frenet { 4531.01, 1.65566 }, Point { 798.284, 2901.47 }), make_tuple(
          Frenet { 4534.4, 1.6412 }, Point { 794.896, 2901.6 }), make_tuple(
          Frenet { 4538.33, 1.65664 }, Point { 790.971, 2901.8 }), make_tuple(
          Frenet { 4542.83, 1.70488 }, Point { 786.472, 2902.05 }), make_tuple(
          Frenet { 4546.92, 1.76926 }, Point { 782.394, 2902.3 }), make_tuple(
          Frenet { 4550.85, 1.83554 }, Point { 778.472, 2902.54 }), make_tuple(
          Frenet { 4554.58, 1.90271 }, Point { 774.742, 2902.78 }), make_tuple(
          Frenet { 4558.15, 1.96167 }, Point { 771.187, 2903 }), make_tuple(
          Frenet { 4561.55, 2.00384 }, Point { 767.788, 2903.19 }), make_tuple(
          Frenet { 4565.39, 2.01832 }, Point { 763.955, 2903.38 }), make_tuple(
          Frenet { 4568.37, 2.0065 }, Point { 760.944, 2903.51 }), make_tuple(
          Frenet { 4571.19, 2.02887 }, Point { 758.129, 2903.61 }), make_tuple(
          Frenet { 4573.58, 2.05044 }, Point { 755.741, 2903.71 }), make_tuple(
          Frenet { 4576.29, 2.08018 }, Point { 753.032, 2903.82 }), make_tuple(
          Frenet { 4578.86, 2.10719 }, Point { 750.464, 2903.93 }), make_tuple(
          Frenet { 4581.65, 2.14077 }, Point { 747.68, 2904.05 }), make_tuple(
          Frenet { 4584.65, 2.17041 }, Point { 744.681, 2904.17 }), make_tuple(
          Frenet { 4587.27, 2.19884 }, Point { 742.056, 2904.28 }), make_tuple(
          Frenet { 4590.35, 2.23232 }, Point { 738.984, 2904.41 }), make_tuple(
          Frenet { 4593.93, 2.26942 }, Point { 735.409, 2904.55 }), make_tuple(
          Frenet { 4597.7, 2.2975 }, Point { 731.637, 2904.7 }), make_tuple(
          Frenet { 4601.68, 2.31142 }, Point { 727.665, 2904.83 }), make_tuple(
          Frenet { 4605.88, 2.30421 }, Point { 723.459, 2904.96 }), make_tuple(
          Frenet { 4609.9, 2.26588 }, Point { 719.447, 2905.04 }), make_tuple(
          Frenet { 4613.96, 2.18997 }, Point { 715.38, 2905.09 }), make_tuple(
          Frenet { 4617.87, 2.07894 }, Point { 711.471, 2905.1 }), make_tuple(
          Frenet { 4621.47, 2.0625 }, Point { 707.726, 2905.07 }), make_tuple(
          Frenet { 4625.05, 2.16011 }, Point { 704.145, 2905.03 }), make_tuple(
          Frenet { 4628.45, 2.23548 }, Point { 700.744, 2904.97 }), make_tuple(
          Frenet { 4631.67, 2.28692 }, Point { 697.523, 2904.9 }), make_tuple(
          Frenet { 4634.72, 2.31613 }, Point { 694.482, 2904.8 }), make_tuple(
          Frenet { 4637.29, 2.33718 }, Point { 691.906, 2904.72 }), make_tuple(
          Frenet { 4640, 2.34584 }, Point { 689.205, 2904.63 }), make_tuple(
          Frenet { 4643, 2.34868 }, Point { 686.202, 2904.51 }), make_tuple(
          Frenet { 4645.6, 2.34525 }, Point { 683.609, 2904.4 }), make_tuple(
          Frenet { 4648.4, 2.34223 }, Point { 680.803, 2904.29 }), make_tuple(
          Frenet { 4651.16, 2.3361 }, Point { 678.051, 2904.18 }), make_tuple(
          Frenet { 4654.4, 2.31899 }, Point { 674.817, 2904.03 }), make_tuple(
          Frenet { 4657.49, 2.29008 }, Point { 671.728, 2903.88 }), make_tuple(
          Frenet { 4660.74, 2.25163 }, Point { 668.479, 2903.71 }), make_tuple(
          Frenet { 4664.15, 2.2046 }, Point { 665.07, 2903.53 }), make_tuple(
          Frenet { 4667.75, 2.15432 }, Point { 661.485, 2903.34 }), make_tuple(
          Frenet { 4671.89, 2.09607 }, Point { 657.348, 2903.12 }), make_tuple(
          Frenet { 4675.83, 2.04196 }, Point { 653.409, 2902.91 }), make_tuple(
          Frenet { 4679.85, 2.05511 }, Point { 649.331, 2902.69 }), make_tuple(
          Frenet { 4683.38, 2.13253 }, Point { 645.801, 2902.52 }), make_tuple(
          Frenet { 4687.49, 2.21917 }, Point { 641.694, 2902.31 }), make_tuple(
          Frenet { 4690.72, 2.28311 }, Point { 638.476, 2902.14 }), make_tuple(
          Frenet { 4694.48, 2.34481 }, Point { 634.717, 2901.93 }), make_tuple(
          Frenet { 4697.39, 2.37384 }, Point { 631.816, 2901.75 }), make_tuple(
          Frenet { 4700.74, 2.38435 }, Point { 628.466, 2901.52 }), make_tuple(
          Frenet { 4703.33, 2.37063 }, Point { 625.886, 2901.32 }), make_tuple(
          Frenet { 4706.05, 2.33591 }, Point { 623.182, 2901.1 }), make_tuple(
          Frenet { 4708.82, 2.27112 }, Point { 620.424, 2900.83 }), make_tuple(
          Frenet { 4710.92, 2.20168 }, Point { 618.33, 2900.61 }), make_tuple(
          Frenet { 4713.44, 2.09468 }, Point { 615.826, 2900.33 }), make_tuple(
          Frenet { 4716.19, 2.0533 }, Point { 612.883, 2899.95 }), make_tuple(
          Frenet { 4718.77, 2.15752 }, Point { 610.328, 2899.61 }), make_tuple(
          Frenet { 4721.81, 2.26289 }, Point { 607.313, 2899.18 }), make_tuple(
          Frenet { 4725.02, 2.34937 }, Point { 604.142, 2898.71 }), make_tuple(
          Frenet { 4728.42, 2.41486 }, Point { 600.779, 2898.18 }), make_tuple(
          Frenet { 4731.65, 2.45693 }, Point { 597.596, 2897.66 }), make_tuple(
          Frenet { 4735.37, 2.4854 }, Point { 593.921, 2897.04 }), make_tuple(
          Frenet { 4739.69, 2.49128 }, Point { 589.668, 2896.29 }), make_tuple(
          Frenet { 4743.4, 2.47788 }, Point { 586.018, 2895.63 }), make_tuple(
          Frenet { 4747.52, 2.43658 }, Point { 581.967, 2894.88 }), make_tuple(
          Frenet { 4751.48, 2.37775 }, Point { 578.08, 2894.13 }), make_tuple(
          Frenet { 4755.24, 2.30834 }, Point { 574.39, 2893.41 }), make_tuple(
          Frenet { 4758.82, 2.22812 }, Point { 570.88, 2892.7 }), make_tuple(
          Frenet { 4762.23, 2.14781 }, Point { 567.53, 2892.03 }), make_tuple(
          Frenet { 4765.45, 2.06471 }, Point { 564.375, 2891.39 }), make_tuple(
          Frenet { 4768.36, 2.0307 }, Point { 561.4, 2890.77 }), make_tuple(
          Frenet { 4771.22, 2.11331 }, Point { 558.601, 2890.17 }), make_tuple(
          Frenet { 4773.37, 2.17033 }, Point { 556.503, 2889.72 }), make_tuple(
          Frenet { 4775.91, 2.23168 }, Point { 554.02, 2889.18 }), make_tuple(
          Frenet { 4778.83, 2.29219 }, Point { 551.171, 2888.54 }), make_tuple(
          Frenet { 4781.67, 2.34789 }, Point { 548.393, 2887.92 }), make_tuple(
          Frenet { 4785, 2.39904 }, Point { 545.145, 2887.18 }), make_tuple(
          Frenet { 4787.92, 2.43334 }, Point { 542.301, 2886.53 }), make_tuple(
          Frenet { 4791.7, 2.45907 }, Point { 538.623, 2885.66 }), make_tuple(
          Frenet { 4795.3, 2.46397 }, Point { 535.124, 2884.81 }), make_tuple(
          Frenet { 4799.08, 2.44759 }, Point { 531.455, 2883.9 }), make_tuple(
          Frenet { 4803.02, 2.41086 }, Point { 527.635, 2882.93 }), make_tuple(
          Frenet { 4806.73, 2.34704 }, Point { 524.048, 2881.99 }), make_tuple(
          Frenet { 4811.06, 2.25032 }, Point { 519.861, 2880.87 }), make_tuple(
          Frenet { 4814.82, 2.14254 }, Point { 516.235, 2879.87 }), make_tuple(
          Frenet { 4818.4, 2.02146 }, Point { 512.787, 2878.91 }), make_tuple(
          Frenet { 4821.3, 2.10556 }, Point { 509.841, 2878.06 }), make_tuple(
          Frenet { 4824.54, 2.20809 }, Point { 506.731, 2877.12 }), make_tuple(
          Frenet { 4827.59, 2.28263 }, Point { 503.818, 2876.23 }), make_tuple(
          Frenet { 4830.76, 2.3381 }, Point { 500.791, 2875.27 }), make_tuple(
          Frenet { 4833.18, 2.36894 }, Point { 498.486, 2874.53 }), make_tuple(
          Frenet { 4835.46, 2.38655 }, Point { 496.32, 2873.82 }), make_tuple(
          Frenet { 4838.04, 2.39857 }, Point { 493.869, 2873.01 }), make_tuple(
          Frenet { 4839.98, 2.39582 }, Point { 492.034, 2872.4 }), make_tuple(
          Frenet { 4842.29, 2.38521 }, Point { 489.844, 2871.65 }), make_tuple(
          Frenet { 4844.77, 2.3609 }, Point { 487.507, 2870.84 }), make_tuple(
          Frenet { 4847.44, 2.32123 }, Point { 484.988, 2869.96 }), make_tuple(
          Frenet { 4850.59, 2.26925 }, Point { 482.016, 2868.91 }), make_tuple(
          Frenet { 4853.33, 2.21472 }, Point { 479.436, 2867.98 }), make_tuple(
          Frenet { 4856.89, 2.1299 }, Point { 476.087, 2866.77 }), make_tuple(
          Frenet { 4860.3, 2.03746 }, Point { 472.878, 2865.6 }), make_tuple(
          Frenet { 4863.78, 2.01753 }, Point { 469.512, 2864.36 }), make_tuple(
          Frenet { 4867.92, 2.1296 }, Point { 465.624, 2862.94 }), make_tuple(
          Frenet { 4872.3, 2.2595 }, Point { 461.506, 2861.45 }), make_tuple(
          Frenet { 4876.82, 2.3963 }, Point { 457.247, 2859.91 }), make_tuple(
          Frenet { 4881.28, 2.52412 }, Point { 453.051, 2858.39 }), make_tuple(
          Frenet { 4885.53, 2.6312 }, Point { 449.059, 2856.93 }), make_tuple(
          Frenet { 4889.23, 2.70617 }, Point { 445.594, 2855.63 }), make_tuple(
          Frenet { 4893.45, 2.76627 }, Point { 441.648, 2854.14 }), make_tuple(
          Frenet { 4896.79, 2.78829 }, Point { 438.534, 2852.93 }), make_tuple(
          Frenet { 4900.28, 2.77895 }, Point { 435.297, 2851.64 }), make_tuple(
          Frenet { 4902.99, 2.74978 }, Point { 432.787, 2850.61 }), make_tuple(
          Frenet { 4906.09, 2.68612 }, Point { 429.923, 2849.41 }), make_tuple(
          Frenet { 4908.52, 2.61245 }, Point { 427.697, 2848.45 }), make_tuple(
          Frenet { 4911.03, 2.51803 }, Point { 425.397, 2847.44 }), make_tuple(
          Frenet { 4913.98, 2.36673 }, Point { 422.703, 2846.21 }), make_tuple(
          Frenet { 4916.56, 2.20563 }, Point { 420.365, 2845.11 }), make_tuple(
          Frenet { 4919.52, 2.00856 }, Point { 417.345, 2843.64 }), make_tuple(
          Frenet { 4922.77, 2.34192 }, Point { 414.436, 2842.17 }), make_tuple(
          Frenet { 4925.84, 2.59523 }, Point { 411.713, 2840.72 }), make_tuple(
          Frenet { 4929.45, 2.81834 }, Point { 408.559, 2838.96 }), make_tuple(
          Frenet { 4933.23, 2.97404 }, Point { 405.298, 2837.05 }), make_tuple(
          Frenet { 4937.21, 3.05796 }, Point { 401.903, 2834.96 }), make_tuple(
          Frenet { 4941.35, 3.06254 }, Point { 398.413, 2832.72 }), make_tuple(
          Frenet { 4945.35, 2.98719 }, Point { 395.089, 2830.5 }), make_tuple(
          Frenet { 4949.15, 2.84564 }, Point { 391.973, 2828.32 }), make_tuple(
          Frenet { 4952.76, 2.65265 }, Point { 389.044, 2826.21 }), make_tuple(
          Frenet { 4956.5, 2.40057 }, Point { 386.035, 2823.98 }), make_tuple(
          Frenet { 4959.4, 2.17232 }, Point { 383.717, 2822.22 }), make_tuple(
          Frenet { 4962.1, 2.05005 }, Point { 381.308, 2820.35 }), make_tuple(
          Frenet { 4964.96, 2.25594 }, Point { 379.057, 2818.57 }), make_tuple(
          Frenet { 4967.9, 2.43673 }, Point { 376.766, 2816.73 }), make_tuple(
          Frenet { 4970.16, 2.55727 }, Point { 375.016, 2815.3 }), make_tuple(
          Frenet { 4972.76, 2.67161 }, Point { 373.015, 2813.63 }), make_tuple(
          Frenet { 4975.02, 2.75815 }, Point { 371.283, 2812.16 }), make_tuple(
          Frenet { 4978.01, 2.85073 }, Point { 369.014, 2810.22 }), make_tuple(
          Frenet { 4981.5, 2.922 }, Point { 366.385, 2807.92 }), make_tuple(
          Frenet { 4984.62, 2.95577 }, Point { 364.06, 2805.84 }), make_tuple(
          Frenet { 4987.9, 2.96457 }, Point { 361.632, 2803.64 }), make_tuple(
          Frenet { 4991.38, 2.93759 }, Point { 359.081, 2801.28 }), make_tuple(
          Frenet { 4995.77, 2.85023 }, Point { 355.892, 2798.25 }), make_tuple(
          Frenet { 4999.98, 2.71081 }, Point { 352.875, 2795.31 }), make_tuple(
          Frenet { 5003.88, 2.53314 }, Point { 350.114, 2792.55 }), make_tuple(
          Frenet { 5007.95, 2.29461 }, Point { 347.266, 2789.63 }), make_tuple(
          Frenet { 5011.46, 2.04132 }, Point { 344.845, 2787.08 }), make_tuple(
          Frenet { 5014.83, 2.16729 }, Point { 342.34, 2784.38 }), make_tuple(
          Frenet { 5018.31, 2.38813 }, Point { 339.986, 2781.8 }), make_tuple(
          Frenet { 5021.59, 2.57727 }, Point { 337.787, 2779.37 }), make_tuple(
          Frenet { 5024.41, 2.7233 }, Point { 335.91, 2777.26 }), make_tuple(
          Frenet { 5027.15, 2.85055 }, Point { 334.091, 2775.2 }), make_tuple(
          Frenet { 5030.06, 2.96676 }, Point { 332.177, 2773.01 }), make_tuple(
          Frenet { 5033.78, 3.08404 }, Point { 329.754, 2770.18 }), make_tuple(
          Frenet { 5037.4, 3.16125 }, Point { 327.428, 2767.41 }), make_tuple(
          Frenet { 5040.88, 3.2027 }, Point { 325.219, 2764.72 }), make_tuple(
          Frenet { 5044.91, 3.21043 }, Point { 322.686, 2761.58 }), make_tuple(
          Frenet { 5048.77, 3.17267 }, Point { 320.301, 2758.55 }), make_tuple(
          Frenet { 5053.07, 3.08178 }, Point { 317.678, 2755.14 }), make_tuple(
          Frenet { 5056.81, 2.95793 }, Point { 315.432, 2752.15 }), make_tuple(
          Frenet { 5060.36, 2.79805 }, Point { 313.33, 2749.28 }), make_tuple(
          Frenet { 5063.43, 2.62559 }, Point { 311.543, 2746.78 }), make_tuple(
          Frenet { 5066.96, 2.38893 }, Point { 309.518, 2743.88 }), make_tuple(
          Frenet { 5070.28, 2.12299 }, Point { 307.642, 2741.12 }), make_tuple(
          Frenet { 5072.81, 2.11017 }, Point { 306.047, 2738.72 }), make_tuple(
          Frenet { 5076.12, 2.34562 }, Point { 304.249, 2735.93 }), make_tuple(
          Frenet { 5078.93, 2.51239 }, Point { 302.751, 2733.54 }), make_tuple(
          Frenet { 5081.33, 2.63321 }, Point { 301.491, 2731.49 }), make_tuple(
          Frenet { 5084.11, 2.74934 }, Point { 300.053, 2729.11 }), make_tuple(
          Frenet { 5086.82, 2.83489 }, Point { 298.675, 2726.78 }), make_tuple(
          Frenet { 5090, 2.90815 }, Point { 297.084, 2724.03 }), make_tuple(
          Frenet { 5093.07, 2.95136 }, Point { 295.57, 2721.36 }), make_tuple(
          Frenet { 5096.68, 2.97028 }, Point { 293.817, 2718.2 }), make_tuple(
          Frenet { 5100.12, 2.95982 }, Point { 292.169, 2715.18 }), make_tuple(
          Frenet { 5103.75, 2.92315 }, Point { 290.457, 2711.98 }), make_tuple(
          Frenet { 5107.15, 2.86931 }, Point { 288.873, 2708.98 }), make_tuple(
          Frenet { 5111.51, 2.77337 }, Point { 286.86, 2705.11 }), make_tuple(
          Frenet { 5115.63, 2.66276 }, Point { 284.973, 2701.44 }), make_tuple(
          Frenet { 5119.57, 2.53879 }, Point { 283.186, 2697.92 }), make_tuple(
          Frenet { 5122.96, 2.42703 }, Point { 281.657, 2694.9 }), make_tuple(
          Frenet { 5126.55, 2.30551 }, Point { 280.036, 2691.69 }), make_tuple(
          Frenet { 5130.28, 2.17444 }, Point { 278.356, 2688.35 }), make_tuple(
          Frenet { 5133.49, 2.06445 }, Point { 276.908, 2685.49 }), make_tuple(
          Frenet { 5136.08, 2.04264 }, Point { 275.674, 2683.05 }), make_tuple(
          Frenet { 5138.96, 2.145 }, Point { 274.377, 2680.48 }), make_tuple(
          Frenet { 5142.57, 2.26344 }, Point { 272.758, 2677.25 }), make_tuple(
          Frenet { 5145.69, 2.35349 }, Point { 271.373, 2674.46 }), make_tuple(
          Frenet { 5148.34, 2.42218 }, Point { 270.202, 2672.08 }), make_tuple(
          Frenet { 5150.56, 2.47282 }, Point { 269.226, 2670.09 }), make_tuple(
          Frenet { 5152.92, 2.52008 }, Point { 268.194, 2667.96 }), make_tuple(
          Frenet { 5155.7, 2.56242 }, Point { 266.993, 2665.45 }), make_tuple(
          Frenet { 5158.67, 2.59228 }, Point { 265.721, 2662.76 }), make_tuple(
          Frenet { 5161.86, 2.61103 }, Point { 264.369, 2659.87 }), make_tuple(
          Frenet { 5164.97, 2.6134 }, Point { 263.068, 2657.06 }), make_tuple(
          Frenet { 5168.25, 2.59843 }, Point { 261.709, 2654.07 }), make_tuple(
          Frenet { 5171.33, 2.5677 }, Point { 260.447, 2651.26 }), make_tuple(
          Frenet { 5175.29, 2.50361 }, Point { 258.847, 2647.63 }), make_tuple(
          Frenet { 5178.68, 2.42683 }, Point { 257.5, 2644.52 }), make_tuple(
          Frenet { 5183, 2.29576 }, Point { 255.812, 2640.55 }), make_tuple(
          Frenet { 5186.7, 2.15014 }, Point { 254.393, 2637.12 }), make_tuple(
          Frenet { 5190.62, 2.03548 }, Point { 252.848, 2633.3 }), make_tuple(
          Frenet { 5194.75, 2.20323 }, Point { 251.347, 2629.45 }), make_tuple(
          Frenet { 5198.87, 2.31703 }, Point { 249.899, 2625.59 }), make_tuple(
          Frenet { 5202.82, 2.37932 }, Point { 248.557, 2621.88 }), make_tuple(
          Frenet { 5206.6, 2.40462 }, Point { 247.304, 2618.31 }), make_tuple(
          Frenet { 5210.52, 2.39769 }, Point { 246.034, 2614.59 }), make_tuple(
          Frenet { 5213.59, 2.37324 }, Point { 245.062, 2611.69 }), make_tuple(
          Frenet { 5216.81, 2.33448 }, Point { 244.051, 2608.63 }), make_tuple(
          Frenet { 5219.85, 2.29022 }, Point { 243.104, 2605.74 }), make_tuple(
          Frenet { 5222.42, 2.24959 }, Point { 242.305, 2603.29 }), make_tuple(
          Frenet { 5225.39, 2.19971 }, Point { 241.388, 2600.47 }), make_tuple(
          Frenet { 5227.66, 2.16106 }, Point { 240.687, 2598.31 }), make_tuple(
          Frenet { 5230, 2.12197 }, Point { 239.963, 2596.09 }), make_tuple(
          Frenet { 5232.37, 2.08729 }, Point { 239.225, 2593.83 }), make_tuple(
          Frenet { 5234.71, 2.0585 }, Point { 238.492, 2591.61 }), make_tuple(
          Frenet { 5237.47, 2.03078 }, Point { 237.62, 2588.99 }), make_tuple(
          Frenet { 5240.18, 2.01187 }, Point { 236.758, 2586.43 }), make_tuple(
          Frenet { 5242.76, 2.00492 }, Point { 235.926, 2583.98 }), make_tuple(
          Frenet { 5246.12, 2.00366 }, Point { 234.83, 2580.82 }), make_tuple(
          Frenet { 5249.01, 2.00023 }, Point { 233.881, 2578.09 }), make_tuple(
          Frenet { 5252.73, 2.00246 }, Point { 232.649, 2574.57 }), make_tuple(
          Frenet { 5256.3, 2.00934 }, Point { 231.466, 2571.2 }), make_tuple(
          Frenet { 5260.07, 2.02 }, Point { 230.215, 2567.65 }), make_tuple(
          Frenet { 5263.99, 2.03489 }, Point { 228.906, 2563.95 }), make_tuple(
          Frenet { 5267.69, 2.04736 }, Point { 227.676, 2560.47 }), make_tuple(
          Frenet { 5272.21, 2.06233 }, Point { 226.172, 2556.2 }), make_tuple(
          Frenet { 5276.73, 2.07135 }, Point { 224.673, 2551.94 }), make_tuple(
          Frenet { 5281.25, 2.07344 }, Point { 223.181, 2547.67 }), make_tuple(
          Frenet { 5285.32, 2.06777 }, Point { 221.843, 2543.82 }), make_tuple(
          Frenet { 5289.58, 2.05194 }, Point { 220.455, 2539.8 }), make_tuple(
          Frenet { 5292.92, 2.02997 }, Point { 219.373, 2536.64 }), make_tuple(
          Frenet { 5296.78, 2.00404 }, Point { 218.123, 2532.95 }), make_tuple(
          Frenet { 5300.47, 2.04486 }, Point { 216.943, 2529.44 }), make_tuple(
          Frenet { 5303.35, 2.07533 }, Point { 216.026, 2526.72 }), make_tuple(
          Frenet { 5306.38, 2.10866 }, Point { 215.057, 2523.84 }), make_tuple(
          Frenet { 5309.22, 2.13723 }, Point { 214.153, 2521.15 }), make_tuple(
          Frenet { 5312.15, 2.16455 }, Point { 213.222, 2518.37 }), make_tuple(
          Frenet { 5314.4, 2.18445 }, Point { 212.508, 2516.24 }), make_tuple(
          Frenet { 5316.74, 2.2015 }, Point { 211.769, 2514.01 }), make_tuple(
          Frenet { 5318.89, 2.21578 }, Point { 211.093, 2511.98 }), make_tuple(
          Frenet { 5321.21, 2.22733 }, Point { 210.364, 2509.77 }), make_tuple(
          Frenet { 5323.72, 2.23556 }, Point { 209.583, 2507.39 }), make_tuple(
          Frenet { 5326.38, 2.24045 }, Point { 208.756, 2504.86 }), make_tuple(
          Frenet { 5329.55, 2.24374 }, Point { 207.776, 2501.85 }), make_tuple(
          Frenet { 5332.28, 2.24058 }, Point { 206.934, 2499.25 }), make_tuple(
          Frenet { 5335.84, 2.22747 }, Point { 205.847, 2495.86 }), make_tuple(
          Frenet { 5338.9, 2.20785 }, Point { 204.922, 2492.95 }), make_tuple(
          Frenet { 5342.85, 2.16699 }, Point { 203.74, 2489.17 }), make_tuple(
          Frenet { 5347, 2.10348 }, Point { 202.519, 2485.21 }), make_tuple(
          Frenet { 5350.56, 2.03238 }, Point { 201.487, 2481.8 }), make_tuple(
          Frenet { 5354.99, 2.0767 }, Point { 200.199, 2477.46 }), make_tuple(
          Frenet { 5359.07, 2.14988 }, Point { 199.055, 2473.55 }), make_tuple(
          Frenet { 5362.98, 2.20449 }, Point { 197.972, 2469.78 }), make_tuple(
          Frenet { 5366.36, 2.24021 }, Point { 197.048, 2466.53 }), make_tuple(
          Frenet { 5369.96, 2.2677 }, Point { 196.076, 2463.07 }), make_tuple(
          Frenet { 5373.7, 2.28472 }, Point { 195.076, 2459.47 }), make_tuple(
          Frenet { 5376.91, 2.28904 }, Point { 194.225, 2456.36 }), make_tuple(
          Frenet { 5379.95, 2.28739 }, Point { 193.427, 2453.43 }), make_tuple(
          Frenet { 5382.8, 2.28036 }, Point { 192.686, 2450.69 }), make_tuple(
          Frenet { 5385.46, 2.26852 }, Point { 191.996, 2448.11 }), make_tuple(
          Frenet { 5387.72, 2.25355 }, Point { 191.418, 2445.93 }), make_tuple(
          Frenet { 5390.11, 2.23449 }, Point { 190.806, 2443.62 }), make_tuple(
          Frenet { 5392.65, 2.20984 }, Point { 190.162, 2441.16 }), make_tuple(
          Frenet { 5395.33, 2.18008 }, Point { 189.485, 2438.57 }), make_tuple(
          Frenet { 5398.15, 2.14556 }, Point { 188.775, 2435.84 }), make_tuple(
          Frenet { 5401.8, 2.09558 }, Point { 187.865, 2432.31 }), make_tuple(
          Frenet { 5405.68, 2.03632 }, Point { 186.902, 2428.55 }), make_tuple(
          Frenet { 5409.73, 2.03182 }, Point { 185.881, 2424.55 }), make_tuple(
          Frenet { 5413.28, 2.10662 }, Point { 185.003, 2421.11 }), make_tuple(
          Frenet { 5417.39, 2.18637 }, Point { 183.993, 2417.12 }), make_tuple(
          Frenet { 5421.88, 2.26071 }, Point { 182.903, 2412.77 }), make_tuple(
          Frenet { 5425.42, 2.30628 }, Point { 182.056, 2409.32 }), make_tuple(
          Frenet { 5429.56, 2.3413 }, Point { 181.084, 2405.3 }), make_tuple(
          Frenet { 5433.47, 2.3492 }, Point { 180.191, 2401.5 }), make_tuple(
          Frenet { 5436.84, 2.33395 }, Point { 179.442, 2398.21 }), make_tuple(
          Frenet { 5440.05, 2.2978 }, Point { 178.749, 2395.08 }), make_tuple(
          Frenet { 5443.06, 2.24182 }, Point { 178.121, 2392.13 }), make_tuple(
          Frenet { 5446.18, 2.15959 }, Point { 177.494, 2389.08 }), make_tuple(
          Frenet { 5449.1, 2.06094 }, Point { 176.928, 2386.21 }), make_tuple(
          Frenet { 5451.19, 2.02286 }, Point { 176.51, 2384.01 }), make_tuple(
          Frenet { 5453.49, 2.09913 }, Point { 176.09, 2381.73 }), make_tuple(
          Frenet { 5455.66, 2.16219 }, Point { 175.705, 2379.6 }), make_tuple(
          Frenet { 5457.99, 2.21977 }, Point { 175.301, 2377.31 }), make_tuple(
          Frenet { 5460.24, 2.26764 }, Point { 174.917, 2375.08 }), make_tuple(
          Frenet { 5462.89, 2.31336 }, Point { 174.477, 2372.47 }), make_tuple(
          Frenet { 5465.74, 2.35022 }, Point { 174.016, 2369.66 }), make_tuple(
          Frenet { 5468.77, 2.37583 }, Point { 173.54, 2366.67 }), make_tuple(
          Frenet { 5471.65, 2.38768 }, Point { 173.098, 2363.82 }), make_tuple(
          Frenet { 5475.02, 2.38791 }, Point { 172.596, 2360.49 }), make_tuple(
          Frenet { 5478.58, 2.3728 }, Point { 172.08, 2356.97 }), make_tuple(
          Frenet { 5482.32, 2.33928 }, Point { 171.555, 2353.26 }), make_tuple(
          Frenet { 5486.64, 2.28132 }, Point { 170.969, 2348.99 }), make_tuple(
          Frenet { 5491.57, 2.19146 }, Point { 170.322, 2344.09 }), make_tuple(
          Frenet { 5495.28, 2.10942 }, Point { 169.852, 2340.42 }), make_tuple(
          Frenet { 5498.96, 2.0169 }, Point { 169.394, 2336.76 }), make_tuple(
          Frenet { 5502.76, 2.08787 }, Point { 168.923, 2332.87 }), make_tuple(
          Frenet { 5506.52, 2.17132 }, Point { 168.491, 2329.14 }), make_tuple(
          Frenet { 5510.1, 2.23225 }, Point { 168.099, 2325.58 }), make_tuple(
          Frenet { 5513.88, 2.27663 }, Point { 167.705, 2321.82 }), make_tuple(
          Frenet { 5517.08, 2.29863 }, Point { 167.386, 2318.63 }), make_tuple(
          Frenet { 5520.68, 2.30675 }, Point { 167.045, 2315.05 }), make_tuple(
          Frenet { 5523.49, 2.30204 }, Point { 166.789, 2312.25 }), make_tuple(
          Frenet { 5526.38, 2.28647 }, Point { 166.538, 2309.38 }), make_tuple(
          Frenet { 5529.2, 2.2613 }, Point { 166.301, 2306.56 }), make_tuple(
          Frenet { 5532.25, 2.22408 }, Point { 166.056, 2303.53 }), make_tuple(
          Frenet { 5534.86, 2.18446 }, Point { 165.854, 2300.92 }), make_tuple(
          Frenet { 5538.26, 2.12291 }, Point { 165.6, 2297.53 }), make_tuple(
          Frenet { 5541.51, 2.05353 }, Point { 165.368, 2294.29 }), make_tuple(
          Frenet { 5544.47, 2.00405 }, Point { 165.159, 2291.24 }), make_tuple(
          Frenet { 5548.41, 2.0757 }, Point { 164.912, 2287.3 }), make_tuple(
          Frenet { 5551.8, 2.11632 }, Point { 164.721, 2283.92 }), make_tuple(
          Frenet { 5556.13, 2.143 }, Point { 164.502, 2279.6 }), make_tuple(
          Frenet { 5559.83, 2.14768 }, Point { 164.333, 2275.9 }), make_tuple(
          Frenet { 5563.94, 2.13762 }, Point { 164.16, 2271.79 }), make_tuple(
          Frenet { 5567.88, 2.11743 }, Point { 164.005, 2267.86 }), make_tuple(
          Frenet { 5572, 2.08866 }, Point { 163.85, 2263.74 }), make_tuple(
          Frenet { 5575.58, 2.0613 }, Point { 163.719, 2260.17 }), make_tuple(
          Frenet { 5578.95, 2.03675 }, Point { 163.593, 2256.8 }), make_tuple(
          Frenet { 5582.14, 2.01642 }, Point { 163.471, 2253.61 }), make_tuple(
          Frenet { 5585.15, 2.00208 }, Point { 163.352, 2250.59 }), make_tuple(
          Frenet { 5588.27, 1.99393 }, Point { 163.221, 2247.48 }), make_tuple(
          Frenet { 5590.93, 1.99407 }, Point { 163.103, 2244.82 }), make_tuple(
          Frenet { 5593.42, 1.99872 }, Point { 162.986, 2242.35 }), make_tuple(
          Frenet { 5595.93, 1.9932 }, Point { 162.859, 2239.84 }), make_tuple(
          Frenet { 5597.88, 1.99305 }, Point { 162.758, 2237.9 }), make_tuple(
          Frenet { 5600.44, 1.99703 }, Point { 162.619, 2235.34 }), make_tuple(
          Frenet { 5602.69, 2.00419 }, Point { 162.494, 2233.09 }), make_tuple(
          Frenet { 5605.37, 2.01655 }, Point { 162.341, 2230.42 }), make_tuple(
          Frenet { 5608.51, 2.03443 }, Point { 162.159, 2227.29 }), make_tuple(
          Frenet { 5611.88, 2.05566 }, Point { 161.961, 2223.92 }), make_tuple(
          Frenet { 5615.79, 2.08007 }, Point { 161.731, 2220.02 }), make_tuple(
          Frenet { 5618.9, 2.09678 }, Point { 161.552, 2216.91 }), make_tuple(
          Frenet { 5622.53, 2.11129 }, Point { 161.347, 2213.29 }), make_tuple(
          Frenet { 5625.93, 2.11749 }, Point { 161.163, 2209.9 }), make_tuple(
          Frenet { 5630.27, 2.1108 }, Point { 160.941, 2205.56 }), make_tuple(
          Frenet { 5634.8, 2.08182 }, Point { 160.733, 2201.04 }), make_tuple(
          Frenet { 5638.5, 2.03825 }, Point { 160.582, 2197.33 }), make_tuple(
          Frenet { 5642.14, 2.01708 }, Point { 160.452, 2193.63 }), make_tuple(
          Frenet { 5646.46, 2.08444 }, Point { 160.313, 2189.31 }), make_tuple(
          Frenet { 5650.22, 2.13465 }, Point { 160.201, 2185.56 }), make_tuple(
          Frenet { 5654.15, 2.17781 }, Point { 160.093, 2181.62 }), make_tuple(
          Frenet { 5657.85, 2.20919 }, Point { 160, 2177.93 }), make_tuple(
          Frenet { 5660.71, 2.22693 }, Point { 159.935, 2175.07 }), make_tuple(
          Frenet { 5663.73, 2.23972 }, Point { 159.872, 2172.04 }), make_tuple(
          Frenet { 5666.84, 2.24593 }, Point { 159.815, 2168.93 }), make_tuple(
          Frenet { 5668.99, 2.24577 }, Point { 159.779, 2166.79 }), make_tuple(
          Frenet { 5671.83, 2.2393 }, Point { 159.739, 2163.94 }), make_tuple(
          Frenet { 5674.84, 2.22568 }, Point { 159.703, 2160.94 }), make_tuple(
          Frenet { 5677.71, 2.20512 }, Point { 159.676, 2158.06 }), make_tuple(
          Frenet { 5681.08, 2.17106 }, Point { 159.654, 2154.7 }), make_tuple(
          Frenet { 5685.33, 2.11304 }, Point { 159.642, 2150.44 }), make_tuple(
          Frenet { 5689.5, 2.03911 }, Point { 159.647, 2146.27 }), make_tuple(
          Frenet { 5693.38, 2.03759 }, Point { 159.666, 2142.31 }), make_tuple(
          Frenet { 5697.43, 2.12059 }, Point { 159.68, 2138.26 }), make_tuple(
          Frenet { 5701.69, 2.21103 }, Point { 159.693, 2134.01 }), make_tuple(
          Frenet { 5705.05, 2.28167 }, Point { 159.703, 2130.64 }), make_tuple(
          Frenet { 5708.62, 2.35309 }, Point { 159.718, 2127.07 }), make_tuple(
          Frenet { 5712.02, 2.41555 }, Point { 159.738, 2123.67 }), make_tuple(
          Frenet { 5714.92, 2.46243 }, Point { 159.761, 2120.77 }), make_tuple(
          Frenet { 5718.59, 2.51117 }, Point { 159.801, 2117.09 }), make_tuple(
          Frenet { 5721.44, 2.53939 }, Point { 159.842, 2114.25 }), make_tuple(
          Frenet { 5724.11, 2.55659 }, Point { 159.889, 2111.58 }), make_tuple(
          Frenet { 5726.6, 2.56356 }, Point { 159.942, 2109.09 }), make_tuple(
          Frenet { 5728.9, 2.56129 }, Point { 160, 2106.8 }), make_tuple(
          Frenet { 5731.28, 2.54926 }, Point { 160.07, 2104.41 }), make_tuple(
          Frenet { 5734.13, 2.52088 }, Point { 160.167, 2101.57 }), make_tuple(
          Frenet { 5736.42, 2.48588 }, Point { 160.257, 2099.27 }), make_tuple(
          Frenet { 5739.43, 2.4231 }, Point { 160.393, 2096.27 }), make_tuple(
          Frenet { 5742.05, 2.35128 }, Point { 160.528, 2093.65 }), make_tuple(
          Frenet { 5745.11, 2.24486 }, Point { 160.708, 2090.59 }), make_tuple(
          Frenet { 5748.65, 2.09048 }, Point { 160.948, 2087.05 }), make_tuple(
          Frenet { 5751.82, 2.11694 }, Point { 161.22, 2083.63 }), make_tuple(
          Frenet { 5756.18, 2.34953 }, Point { 161.656, 2079.29 }), make_tuple(
          Frenet { 5760.01, 2.47381 }, Point { 162.12, 2075.48 }), make_tuple(
          Frenet { 5764.02, 2.53496 }, Point { 162.673, 2071.51 }), make_tuple(
          Frenet { 5768.98, 2.53102 }, Point { 163.437, 2066.61 }), make_tuple(
          Frenet { 5773.44, 2.47116 }, Point { 164.178, 2062.22 }), make_tuple(
          Frenet { 5777.33, 2.38826 }, Point { 164.855, 2058.38 }), make_tuple(
          Frenet { 5781.39, 2.28323 }, Point { 165.581, 2054.38 }), make_tuple(
          Frenet { 5785.24, 2.1774 }, Point { 166.274, 2050.6 }), make_tuple(
          Frenet { 5788.9, 2.08086 }, Point { 166.93, 2047 }), make_tuple(
          Frenet { 5792.05, 2.00802 }, Point { 167.484, 2043.89 }), make_tuple(
          Frenet { 5794.95, 2.05015 }, Point { 168.001, 2040.96 }), make_tuple(
          Frenet { 5798.04, 2.1033 }, Point { 168.536, 2037.92 }), make_tuple(
          Frenet { 5800.94, 2.15195 }, Point { 169.039, 2035.07 }), make_tuple(
          Frenet { 5803.47, 2.19254 }, Point { 169.482, 2032.57 }), make_tuple(
          Frenet { 5806.19, 2.23314 }, Point { 169.958, 2029.89 }), make_tuple(
          Frenet { 5808.78, 2.26802 }, Point { 170.417, 2027.34 }), make_tuple(
          Frenet { 5811.82, 2.30409 }, Point { 170.96, 2024.35 }), make_tuple(
          Frenet { 5815.36, 2.33686 }, Point { 171.602, 2020.87 }), make_tuple(
          Frenet { 5819.48, 2.3615 }, Point { 172.36, 2016.82 }), make_tuple(
          Frenet { 5823.1, 2.36713 }, Point { 173.043, 2013.27 }), make_tuple(
          Frenet { 5826.91, 2.35474 }, Point { 173.78, 2009.53 }), make_tuple(
          Frenet { 5830.49, 2.32462 }, Point { 174.491, 2006.02 }), make_tuple(
          Frenet { 5834.61, 2.26526 }, Point { 175.333, 2001.98 }), make_tuple(
          Frenet { 5838.73, 2.17678 }, Point { 176.203, 1997.96 }), make_tuple(
          Frenet { 5843.08, 2.04779 }, Point { 177.158, 1993.71 }), make_tuple(
          Frenet { 5847.4, 2.12569 }, Point { 178.166, 1989.36 }), make_tuple(
          Frenet { 5850.91, 2.26062 }, Point { 178.964, 1985.93 }), make_tuple(
          Frenet { 5854.58, 2.39472 }, Point { 179.803, 1982.35 }), make_tuple(
          Frenet { 5858.04, 2.51152 }, Point { 180.603, 1978.99 }), make_tuple(
          Frenet { 5861.59, 2.61628 }, Point { 181.44, 1975.53 }), make_tuple(
          Frenet { 5864.93, 2.69713 }, Point { 182.243, 1972.29 }), make_tuple(
          Frenet { 5867.77, 2.7498 }, Point { 182.941, 1969.54 }), make_tuple(
          Frenet { 5870.18, 2.77992 }, Point { 183.548, 1967.21 }), make_tuple(
          Frenet { 5872.21, 2.79486 }, Point { 184.07, 1965.24 }), make_tuple(
          Frenet { 5874.4, 2.79917 }, Point { 184.644, 1963.13 }), make_tuple(
          Frenet { 5876.74, 2.78912 }, Point { 185.271, 1960.88 }), make_tuple(
          Frenet { 5879.26, 2.76024 }, Point { 185.963, 1958.45 }), make_tuple(
          Frenet { 5881.67, 2.71357 }, Point { 186.644, 1956.14 }), make_tuple(
          Frenet { 5884.79, 2.62163 }, Point { 187.558, 1953.15 }), make_tuple(
          Frenet { 5887.54, 2.51009 }, Point { 188.391, 1950.53 }), make_tuple(
          Frenet { 5890.76, 2.33919 }, Point { 189.403, 1947.48 }), make_tuple(
          Frenet { 5894.15, 2.1094 }, Point { 190.52, 1944.27 }), make_tuple(
          Frenet { 5897.03, 2.13255 }, Point { 191.627, 1941.25 }), make_tuple(
          Frenet { 5900.78, 2.35974 }, Point { 192.98, 1937.74 }), make_tuple(
          Frenet { 5905.09, 2.53749 }, Point { 194.61, 1933.75 }), make_tuple(
          Frenet { 5909.22, 2.63883 }, Point { 196.235, 1929.95 }), make_tuple(
          Frenet { 5913.74, 2.68042 }, Point { 198.079, 1925.82 }), make_tuple(
          Frenet { 5918.03, 2.66602 }, Point { 199.876, 1921.93 }), make_tuple(
          Frenet { 5921.75, 2.62163 }, Point { 201.461, 1918.57 }), make_tuple(
          Frenet { 5925.3, 2.55852 }, Point { 202.995, 1915.36 }), make_tuple(
          Frenet { 5928.67, 2.48562 }, Point { 204.463, 1912.33 }), make_tuple(
          Frenet { 5931.57, 2.41363 }, Point { 205.735, 1909.72 }), make_tuple(
          Frenet { 5934.91, 2.32787 }, Point { 207.205, 1906.71 }), make_tuple(
          Frenet { 5937.78, 2.25365 }, Point { 208.463, 1904.14 }), make_tuple(
          Frenet { 5940.45, 2.18687 }, Point { 209.638, 1901.73 }), make_tuple(
          Frenet { 5942.7, 2.13533 }, Point { 210.62, 1899.71 }), make_tuple(
          Frenet { 5945.65, 2.07191 }, Point { 211.901, 1897.06 }), make_tuple(
          Frenet { 5948.49, 2.02112 }, Point { 213.132, 1894.49 }), make_tuple(
          Frenet { 5951.81, 2.00238 }, Point { 214.558, 1891.46 }), make_tuple(
          Frenet { 5955.38, 2.02751 }, Point { 216.084, 1888.22 }), make_tuple(
          Frenet { 5959.55, 2.04297 }, Point { 217.876, 1884.46 }), make_tuple(
          Frenet { 5963.2, 2.0471 }, Point { 219.452, 1881.17 }), make_tuple(
          Frenet { 5967.02, 2.0435 }, Point { 221.112, 1877.73 }), make_tuple(
          Frenet { 5970.52, 2.03481 }, Point { 222.637, 1874.58 }), make_tuple(
          Frenet { 5974.61, 2.02239 }, Point { 224.42, 1870.89 }), make_tuple(
          Frenet { 5978.18, 2.01115 }, Point { 225.977, 1867.68 }), make_tuple(
          Frenet { 5981.57, 1.99986 }, Point { 227.456, 1864.63 }), make_tuple(
          Frenet { 5985.09, 1.99064 }, Point { 228.986, 1861.47 }), make_tuple(
          Frenet { 5988.08, 1.98768 }, Point { 230.287, 1858.77 }), make_tuple(
          Frenet { 5990.92, 1.99205 }, Point { 231.511, 1856.21 }), make_tuple(
          Frenet { 5993.82, 2.00443 }, Point { 232.758, 1853.59 }), make_tuple(
          Frenet { 5996.38, 2.00359 }, Point { 233.843, 1851.3 }), make_tuple(
          Frenet { 5999.07, 1.99117 }, Point { 235.002, 1848.86 }), make_tuple(
          Frenet { 6001.65, 1.97575 }, Point { 236.109, 1846.54 }), make_tuple(
          Frenet { 6004.38, 1.9569 }, Point { 237.289, 1844.07 }), make_tuple(
          Frenet { 6007.26, 1.93554 }, Point { 238.531, 1841.48 }), make_tuple(
          Frenet { 6010.26, 1.91166 }, Point { 239.828, 1838.77 }), make_tuple(
          Frenet { 6013.76, 1.88356 }, Point { 241.341, 1835.62 }), make_tuple(
          Frenet { 6017.81, 1.85263 }, Point { 243.092, 1831.96 }), make_tuple(
          Frenet { 6022.5, 1.82451 }, Point { 245.113, 1827.73 }), make_tuple(
          Frenet { 6026.19, 1.80649 }, Point { 246.699, 1824.39 }), make_tuple(
          Frenet { 6030.3, 1.79405 }, Point { 248.454, 1820.68 }), make_tuple(
          Frenet { 6034.36, 1.79367 }, Point { 250.182, 1817.01 }), make_tuple(
          Frenet { 6038.26, 1.80587 }, Point { 251.83, 1813.47 }), make_tuple(
          Frenet { 6041.63, 1.82798 }, Point { 253.242, 1810.41 }), make_tuple(
          Frenet { 6045.23, 1.86636 }, Point { 254.737, 1807.14 }), make_tuple(
          Frenet { 6047.95, 1.90822 }, Point { 255.857, 1804.66 }), make_tuple(
          Frenet { 6051.21, 1.96883 }, Point { 257.187, 1801.68 }), make_tuple(
          Frenet { 6054.07, 1.97056 }, Point { 258.31, 1799.14 }), make_tuple(
          Frenet { 6056.71, 1.91679 }, Point { 259.368, 1796.73 }), make_tuple(
          Frenet { 6059.19, 1.87243 }, Point { 260.357, 1794.46 }), make_tuple(
          Frenet { 6062.02, 1.82936 }, Point { 261.481, 1791.86 }), make_tuple(
          Frenet { 6064.83, 1.79383 }, Point { 262.591, 1789.27 }), make_tuple(
          Frenet { 6066.76, 1.77401 }, Point { 263.346, 1787.5 }), make_tuple(
          Frenet { 6068.84, 1.75502 }, Point { 264.161, 1785.59 }), make_tuple(
          Frenet { 6071.09, 1.73848 }, Point { 265.036, 1783.52 }), make_tuple(
          Frenet { 6073.49, 1.7231 }, Point { 265.971, 1781.3 }), make_tuple(
          Frenet { 6076.31, 1.71146 }, Point { 267.06, 1778.7 }), make_tuple(
          Frenet { 6078.97, 1.70736 }, Point { 268.081, 1776.25 }), make_tuple(
          Frenet { 6082.1, 1.70992 }, Point { 269.28, 1773.35 }), make_tuple(
          Frenet { 6084.76, 1.7187 }, Point { 270.289, 1770.89 }), make_tuple(
          Frenet { 6088.26, 1.73857 }, Point { 271.609, 1767.65 }), make_tuple(
          Frenet { 6092.33, 1.76928 }, Point { 273.137, 1763.88 }), make_tuple(
          Frenet { 6096.61, 1.81257 }, Point { 274.737, 1759.91 }), make_tuple(
          Frenet { 6100.3, 1.85687 }, Point { 276.11, 1756.48 }), make_tuple(
          Frenet { 6104.41, 1.90754 }, Point { 277.634, 1752.67 }), make_tuple(
          Frenet { 6109.28, 1.96975 }, Point { 279.441, 1748.14 }), make_tuple(
          Frenet { 6112.84, 1.97191 }, Point { 280.741, 1744.88 }), make_tuple(
          Frenet { 6115.46, 1.94703 }, Point { 281.704, 1742.45 }), make_tuple(
          Frenet { 6119.08, 1.92385 }, Point { 283.023, 1739.08 }), make_tuple(
          Frenet { 6122.5, 1.91252 }, Point { 284.262, 1735.89 }), make_tuple(
          Frenet { 6126.06, 1.91088 }, Point { 285.543, 1732.56 }), make_tuple(
          Frenet { 6129.15, 1.91649 }, Point { 286.644, 1729.68 }), make_tuple(
          Frenet { 6132.29, 1.92604 }, Point { 287.762, 1726.75 }), make_tuple(
          Frenet { 6134.68, 1.93668 }, Point { 288.61, 1724.51 }), make_tuple(
          Frenet { 6136.93, 1.94691 }, Point { 289.407, 1722.41 }), make_tuple(
          Frenet { 6139.34, 1.96035 }, Point { 290.259, 1720.15 }), make_tuple(
          Frenet { 6142.16, 1.97819 }, Point { 291.254, 1717.52 }), make_tuple(
          Frenet { 6144.85, 1.99443 }, Point { 292.206, 1714.99 }), make_tuple(
          Frenet { 6148, 2.01164 }, Point { 293.318, 1712.05 }), make_tuple(
          Frenet { 6151.34, 2.02526 }, Point { 294.504, 1708.93 }), make_tuple(
          Frenet { 6154.88, 2.03654 }, Point { 295.763, 1705.62 }), make_tuple(
          Frenet { 6158.61, 2.04133 }, Point { 297.099, 1702.13 }), make_tuple(
          Frenet { 6161.74, 2.03719 }, Point { 298.225, 1699.21 }), make_tuple(
          Frenet { 6165.82, 2.02027 }, Point { 299.704, 1695.42 }), make_tuple(
          Frenet { 6170.3, 2.02657 }, Point { 301.357, 1691.21 }), make_tuple(
          Frenet { 6174.64, 2.08055 }, Point { 302.942, 1687.17 }), make_tuple(
          Frenet { 6178.39, 2.12674 }, Point { 304.313, 1683.68 }), make_tuple(
          Frenet { 6181.93, 2.16828 }, Point { 305.609, 1680.39 }), make_tuple(
          Frenet { 6184.96, 2.20066 }, Point { 306.725, 1677.56 }), make_tuple(
          Frenet { 6187.54, 2.22367 }, Point { 307.675, 1675.17 }), make_tuple(
          Frenet { 6190.62, 2.24446 }, Point { 308.816, 1672.31 }), make_tuple(
          Frenet { 6193.25, 2.25501 }, Point { 309.798, 1669.87 }), make_tuple(
          Frenet { 6195.74, 2.25797 }, Point { 310.734, 1667.56 }), make_tuple(
          Frenet { 6197.83, 2.25355 }, Point { 311.528, 1665.62 }), make_tuple(
          Frenet { 6200.27, 2.23885 }, Point { 312.46, 1663.37 }), make_tuple(
          Frenet { 6202.92, 2.21235 }, Point { 313.484, 1660.93 }), make_tuple(
          Frenet { 6205.45, 2.1738 }, Point { 314.472, 1658.6 }), make_tuple(
          Frenet { 6208.24, 2.11464 }, Point { 315.581, 1656.03 }), make_tuple(
          Frenet { 6211.29, 2.02909 }, Point { 316.809, 1653.24 }), make_tuple(
          Frenet { 6213.81, 2.05351 }, Point { 317.895, 1650.82 }), make_tuple(
          Frenet { 6217.21, 2.13772 }, Point { 319.309, 1647.73 }), make_tuple(
          Frenet { 6219.79, 2.18781 }, Point { 320.393, 1645.39 }), make_tuple(
          Frenet { 6222.51, 2.22714 }, Point { 321.549, 1642.92 }), make_tuple(
          Frenet { 6226.04, 2.26228 }, Point { 323.062, 1639.73 }), make_tuple(
          Frenet { 6229.73, 2.27687 }, Point { 324.664, 1636.41 }), make_tuple(
          Frenet { 6233.22, 2.27391 }, Point { 326.195, 1633.27 }), make_tuple(
          Frenet { 6237.25, 2.25317 }, Point { 327.976, 1629.66 }), make_tuple(
          Frenet { 6241.77, 2.21257 }, Point { 329.995, 1625.61 }), make_tuple(
          Frenet { 6245.89, 2.16213 }, Point { 331.84, 1621.93 }), make_tuple(
          Frenet { 6249.59, 2.10706 }, Point { 333.51, 1618.63 }), make_tuple(
          Frenet { 6253.53, 2.03948 }, Point { 335.299, 1615.11 }), make_tuple(
          Frenet { 6257.22, 2.00849 }, Point { 336.999, 1611.77 }), make_tuple(
          Frenet { 6260.8, 2.05261 }, Point { 338.632, 1608.59 }), make_tuple(
          Frenet { 6264.55, 2.08837 }, Point { 340.355, 1605.25 }), make_tuple(
          Frenet { 6268.36, 2.11078 }, Point { 342.115, 1601.87 }), make_tuple(
          Frenet { 6271.06, 2.12056 }, Point { 343.368, 1599.48 }), make_tuple(
          Frenet { 6273.88, 2.1263 }, Point { 344.683, 1596.98 }), make_tuple(
          Frenet { 6276.53, 2.12791 }, Point { 345.917, 1594.64 }), make_tuple(
          Frenet { 6279.24, 2.12547 }, Point { 347.189, 1592.24 }), make_tuple(
          Frenet { 6281.82, 2.1208 }, Point { 348.396, 1589.97 }), make_tuple(
          Frenet { 6284.08, 2.11354 }, Point { 349.462, 1587.97 }), make_tuple(
          Frenet { 6286.78, 2.10056 }, Point { 350.735, 1585.59 }), make_tuple(
          Frenet { 6289.96, 2.08328 }, Point { 352.236, 1582.79 }), make_tuple(
          Frenet { 6293.02, 2.06605 }, Point { 353.679, 1580.09 }), make_tuple(
          Frenet { 6295.59, 2.04975 }, Point { 354.895, 1577.83 }), make_tuple(
          Frenet { 6298.94, 2.03264 }, Point { 356.479, 1574.87 }), make_tuple(
          Frenet { 6303.23, 2.01148 }, Point { 358.501, 1571.09 }), make_tuple(
          Frenet { 6306.98, 1.99408 }, Point { 360.268, 1567.79 }), make_tuple(
          Frenet { 6311.29, 2.00457 }, Point { 362.311, 1563.96 }), make_tuple(
          Frenet { 6315.4, 2.03458 }, Point { 364.242, 1560.34 }), make_tuple(
          Frenet { 6319.06, 2.06402 }, Point { 365.961, 1557.11 }), make_tuple(
          Frenet { 6322.59, 2.09391 }, Point { 367.621, 1553.99 }), make_tuple(
          Frenet { 6325.61, 2.11811 }, Point { 369.039, 1551.32 }), make_tuple(
          Frenet { 6329.24, 2.14309 }, Point { 370.747, 1548.12 }), make_tuple(
          Frenet { 6332.7, 2.16103 }, Point { 372.384, 1545.07 }), make_tuple(
          Frenet { 6335.99, 2.17119 }, Point { 373.944, 1542.17 }), make_tuple(
          Frenet { 6339.39, 2.17337 }, Point { 375.562, 1539.18 }), make_tuple(
          Frenet { 6342.52, 2.16636 }, Point { 377.06, 1536.43 }), make_tuple(
          Frenet { 6345.17, 2.153 }, Point { 378.335, 1534.11 }), make_tuple(
          Frenet { 6347.4, 2.13522 }, Point { 379.417, 1532.16 }), make_tuple(
          Frenet { 6349.75, 2.11055 }, Point { 380.556, 1530.11 }), make_tuple(
          Frenet { 6352, 2.07936 }, Point { 381.659, 1528.14 }), make_tuple(
          Frenet { 6354.67, 2.03188 }, Point { 382.972, 1525.82 }), make_tuple(
          Frenet { 6357.13, 1.99767 }, Point { 384.233, 1523.61 }), make_tuple(
          Frenet { 6360.12, 2.04269 }, Point { 385.722, 1521.02 }), make_tuple(
          Frenet { 6363.31, 2.07834 }, Point { 387.319, 1518.27 }), make_tuple(
          Frenet { 6366.67, 2.10697 }, Point { 389.014, 1515.36 }), make_tuple(
          Frenet { 6370.23, 2.1234 }, Point { 390.82, 1512.29 }), make_tuple(
          Frenet { 6373.97, 2.12941 }, Point { 392.725, 1509.08 }), make_tuple(
          Frenet { 6378.31, 2.12509 }, Point { 394.95, 1505.35 }), make_tuple(
          Frenet { 6382.01, 2.11469 }, Point { 396.85, 1502.17 }), make_tuple(
          Frenet { 6386.12, 2.09705 }, Point { 398.966, 1498.65 }), make_tuple(
          Frenet { 6390.23, 2.0737 }, Point { 401.086, 1495.13 }), make_tuple(
          Frenet { 6393.76, 2.04964 }, Point { 402.914, 1492.11 }), make_tuple(
          Frenet { 6397.9, 2.02307 }, Point { 405.054, 1488.56 }), make_tuple(
          Frenet { 6401.49, 2.00445 }, Point { 406.904, 1485.49 }), make_tuple(
          Frenet { 6405.52, 1.99148 }, Point { 408.976, 1482.03 }), make_tuple(
          Frenet { 6408.69, 1.98804 }, Point { 410.602, 1479.31 }), make_tuple(
          Frenet { 6411.67, 1.99191 }, Point { 412.121, 1476.74 }), make_tuple(
          Frenet { 6415.02, 2.00635 }, Point { 413.821, 1473.86 }), make_tuple(
          Frenet { 6417.65, 1.99561 }, Point { 415.14, 1471.61 }), make_tuple(
          Frenet { 6419.88, 1.98061 }, Point { 416.271, 1469.69 }), make_tuple(
          Frenet { 6422.22, 1.96294 }, Point { 417.457, 1467.67 }), make_tuple(
          Frenet { 6425.27, 1.94036 }, Point { 419.006, 1465.04 }), make_tuple(
          Frenet { 6428.25, 1.91766 }, Point { 420.518, 1462.47 }), make_tuple(
          Frenet { 6430.54, 1.90313 }, Point { 421.675, 1460.5 }), make_tuple(
          Frenet { 6433.59, 1.88509 }, Point { 423.219, 1457.87 }), make_tuple(
          Frenet { 6436.48, 1.87163 }, Point { 424.679, 1455.38 }), make_tuple(
          Frenet { 6439.87, 1.85884 }, Point { 426.387, 1452.45 }), make_tuple(
          Frenet { 6443.4, 1.8527 }, Point { 428.162, 1449.39 }), make_tuple(
          Frenet { 6446.34, 1.85325 }, Point { 429.635, 1446.85 }), make_tuple(
          Frenet { 6450.21, 1.86099 }, Point { 431.566, 1443.49 }), make_tuple(
          Frenet { 6454.26, 1.87917 }, Point { 433.577, 1439.98 }), make_tuple(
          Frenet { 6457.82, 1.90539 }, Point { 435.34, 1436.89 }), make_tuple(
          Frenet { 6461.62, 1.94541 }, Point { 437.206, 1433.58 }), make_tuple(
          Frenet { 6465.29, 1.98881 }, Point { 438.974, 1430.43 }), make_tuple(
          Frenet { 6468.69, 1.95251 }, Point { 440.618, 1427.45 }), make_tuple(
          Frenet { 6471.91, 1.93286 }, Point { 442.165, 1424.63 }), make_tuple(
          Frenet { 6474.96, 1.92921 }, Point { 443.614, 1421.95 }), make_tuple(
          Frenet { 6478.08, 1.93384 }, Point { 445.09, 1419.2 }), make_tuple(
          Frenet { 6481.34, 1.94686 }, Point { 446.626, 1416.32 }), make_tuple(
          Frenet { 6483.94, 1.96316 }, Point { 447.848, 1414.02 }), make_tuple(
          Frenet { 6487.01, 1.98643 }, Point { 449.284, 1411.31 }), make_tuple(
          Frenet { 6490.94, 2.02223 }, Point { 451.117, 1407.84 }), make_tuple(
          Frenet { 6494.74, 2.06062 }, Point { 452.889, 1404.47 }), make_tuple(
          Frenet { 6498.73, 2.09573 }, Point { 454.749, 1400.95 }), make_tuple(
          Frenet { 6501.77, 2.11484 }, Point { 456.175, 1398.26 }), make_tuple(
          Frenet { 6505.74, 2.12708 }, Point { 458.049, 1394.76 }), make_tuple(
          Frenet { 6509.85, 2.12307 }, Point { 460.006, 1391.14 }), make_tuple(
          Frenet { 6514.19, 2.0949 }, Point { 462.087, 1387.34 }), make_tuple(
          Frenet { 6517.95, 2.04537 }, Point { 463.916, 1384.05 }), make_tuple(
          Frenet { 6521.79, 2.04063 }, Point { 465.845, 1380.64 }), make_tuple(
          Frenet { 6525.79, 2.11114 }, Point { 467.827, 1377.16 }), make_tuple(
          Frenet { 6528.93, 2.15855 }, Point { 469.39, 1374.43 }), make_tuple(
          Frenet { 6532.2, 2.20084 }, Point { 471.022, 1371.6 }), make_tuple(
          Frenet { 6534.99, 2.23205 }, Point { 472.419, 1369.18 }), make_tuple(
          Frenet { 6537.62, 2.25736 }, Point { 473.737, 1366.91 }), make_tuple(
          Frenet { 6540.34, 2.28034 }, Point { 475.104, 1364.56 }), make_tuple(
          Frenet { 6542.41, 2.29602 }, Point { 476.149, 1362.77 }), make_tuple(
          Frenet { 6544.56, 2.30943 }, Point { 477.233, 1360.91 }), make_tuple(
          Frenet { 6546.89, 2.31824 }, Point { 478.411, 1358.91 }), make_tuple(
          Frenet { 6549.39, 2.32073 }, Point { 479.686, 1356.76 }), make_tuple(
          Frenet { 6552.07, 2.32066 }, Point { 481.055, 1354.45 }), make_tuple(
          Frenet { 6554.34, 2.31754 }, Point { 482.216, 1352.5 }), make_tuple(
          Frenet { 6557.05, 2.30869 }, Point { 483.602, 1350.18 }), make_tuple(
          Frenet { 6560.53, 2.28783 }, Point { 485.399, 1347.19 }), make_tuple(
          Frenet { 6563.22, 2.26449 }, Point { 486.787, 1344.89 }), make_tuple(
          Frenet { 6566.72, 2.2228 }, Point { 488.611, 1341.9 }), make_tuple(
          Frenet { 6570.4, 2.16238 }, Point { 490.542, 1338.76 }), make_tuple(
          Frenet { 6574.69, 2.07418 }, Point { 492.807, 1335.12 }), make_tuple(
          Frenet { 6578.3, 2.02932 }, Point { 494.769, 1331.98 }), make_tuple(
          Frenet { 6582, 2.1086 }, Point { 496.738, 1328.85 }), make_tuple(
          Frenet { 6586.11, 2.185 }, Point { 498.935, 1325.37 }), make_tuple(
          Frenet { 6590.45, 2.2599 }, Point { 501.264, 1321.7 }), make_tuple(
          Frenet { 6593.83, 2.30936 }, Point { 503.082, 1318.85 }), make_tuple(
          Frenet { 6597.45, 2.354 }, Point { 505.033, 1315.81 }), make_tuple(
          Frenet { 6600.86, 2.38564 }, Point { 506.887, 1312.94 }), make_tuple(
          Frenet { 6604.39, 2.40352 }, Point { 508.811, 1309.99 }), make_tuple(
          Frenet { 6607.43, 2.40757 }, Point { 510.482, 1307.44 }), make_tuple(
          Frenet { 6610.28, 2.40312 }, Point { 512.052, 1305.07 }), make_tuple(
          Frenet { 6612.69, 2.39365 }, Point { 513.389, 1303.06 }), make_tuple(
          Frenet { 6615.2, 2.37525 }, Point { 514.783, 1300.98 }), make_tuple(
          Frenet { 6617.88, 2.34818 }, Point { 516.282, 1298.75 }), make_tuple(
          Frenet { 6620.44, 2.31215 }, Point { 517.72, 1296.63 }), make_tuple(
          Frenet { 6623.15, 2.26428 }, Point { 519.247, 1294.4 }), make_tuple(
          Frenet { 6626.63, 2.18713 }, Point { 521.231, 1291.53 }), make_tuple(
          Frenet { 6630.35, 2.09076 }, Point { 523.359, 1288.48 }), make_tuple(
          Frenet { 6633.93, 1.98255 }, Point { 525.418, 1285.55 }), make_tuple(
          Frenet { 6636.79, 2.08396 }, Point { 527.151, 1283.1 }), make_tuple(
          Frenet { 6640.7, 2.22499 }, Point { 529.415, 1279.91 }), make_tuple(
          Frenet { 6644.82, 2.36626 }, Point { 531.801, 1276.55 }), make_tuple(
          Frenet { 6648.53, 2.48525 }, Point { 533.959, 1273.53 }), make_tuple(
          Frenet { 6652.13, 2.58216 }, Point { 536.07, 1270.61 }), make_tuple(
          Frenet { 6655.97, 2.66541 }, Point { 538.336, 1267.51 }), make_tuple(
          Frenet { 6659.24, 2.71796 }, Point { 540.279, 1264.88 }), make_tuple(
          Frenet { 6663.05, 2.75227 }, Point { 542.563, 1261.84 }), make_tuple(
          Frenet { 6666.32, 2.75478 }, Point { 544.547, 1259.24 }), make_tuple(
          Frenet { 6669.68, 2.72749 }, Point { 546.608, 1256.58 }), make_tuple(
          Frenet { 6672.53, 2.68194 }, Point { 548.379, 1254.34 }), make_tuple(
          Frenet { 6675.19, 2.61471 }, Point { 550.045, 1252.27 }), make_tuple(
          Frenet { 6677.92, 2.51915 }, Point { 551.778, 1250.16 }), make_tuple(
          Frenet { 6680.43, 2.406 }, Point { 553.393, 1248.24 }), make_tuple(
          Frenet { 6682.61, 2.28472 }, Point { 554.817, 1246.57 }), make_tuple(
          Frenet { 6684.98, 2.13019 }, Point { 556.376, 1244.79 }), make_tuple(
          Frenet { 6687.48, 2.03966 }, Point { 558.255, 1242.68 }), make_tuple(
          Frenet { 6689.36, 2.17186 }, Point { 559.527, 1241.29 }), make_tuple(
          Frenet { 6692.22, 2.34653 }, Point { 561.474, 1239.2 }), make_tuple(
          Frenet { 6695.23, 2.49251 }, Point { 563.559, 1237.01 }), make_tuple(
          Frenet { 6698.43, 2.60311 }, Point { 565.799, 1234.72 }), make_tuple(
          Frenet { 6701.47, 2.67163 }, Point { 567.952, 1232.58 }), make_tuple(
          Frenet { 6704.27, 2.70815 }, Point { 569.958, 1230.62 }), make_tuple(
          Frenet { 6708, 2.72297 }, Point { 572.644, 1228.04 }), make_tuple(
          Frenet { 6711.5, 2.70026 }, Point { 575.198, 1225.64 }), make_tuple(
          Frenet { 6715.98, 2.62756 }, Point { 578.489, 1222.61 }), make_tuple(
          Frenet { 6720.93, 2.5039 }, Point { 582.163, 1219.29 }), make_tuple(
          Frenet { 6724.64, 2.39402 }, Point { 584.927, 1216.81 }), make_tuple(
          Frenet { 6729.17, 2.23955 }, Point { 588.313, 1213.8 }), make_tuple(
          Frenet { 6733.22, 2.08976 }, Point { 591.353, 1211.11 }), make_tuple(
          Frenet { 6737.33, 2.07161 }, Point { 594.549, 1208.29 }), make_tuple(
          Frenet { 6741.06, 2.21719 }, Point { 597.349, 1205.81 }), make_tuple(
          Frenet { 6744.61, 2.34713 }, Point { 600.012, 1203.47 }), make_tuple(
          Frenet { 6748.32, 2.47023 }, Point { 602.809, 1201.03 }), make_tuple(
          Frenet { 6751.51, 2.56362 }, Point { 605.226, 1198.94 }), make_tuple(
          Frenet { 6754.26, 2.62996 }, Point { 607.313, 1197.15 }), make_tuple(
          Frenet { 6756.56, 2.67824 }, Point { 609.065, 1195.66 }), make_tuple(
          Frenet { 6759.3, 2.71584 }, Point { 611.159, 1193.9 }), make_tuple(
          Frenet { 6762.1, 2.73662 }, Point { 613.32, 1192.11 }), make_tuple(
          Frenet { 6764.48, 2.73861 }, Point { 615.159, 1190.61 }), make_tuple(
          Frenet { 6766.89, 2.72056 }, Point { 617.038, 1189.1 }), make_tuple(
          Frenet { 6769.01, 2.6886 }, Point { 618.703, 1187.78 }), make_tuple(
          Frenet { 6771.81, 2.61391 }, Point { 620.924, 1186.07 }), make_tuple(
          Frenet { 6774.53, 2.50855 }, Point { 623.093, 1184.43 }), make_tuple(
          Frenet { 6777.42, 2.36377 }, Point { 625.426, 1182.72 }), make_tuple(
          Frenet { 6780.47, 2.16388 }, Point { 627.916, 1180.94 }), make_tuple(
          Frenet { 6784.05, 2.11762 }, Point { 631.166, 1178.72 }), make_tuple(
          Frenet { 6787.13, 2.33917 }, Point { 633.745, 1177.02 }), make_tuple(
          Frenet { 6790.74, 2.55025 }, Point { 636.789, 1175.07 }), make_tuple(
          Frenet { 6794.54, 2.71781 }, Point { 640.016, 1173.06 }), make_tuple(
          Frenet { 6798.52, 2.8329 }, Point { 643.431, 1171.01 }), make_tuple(
          Frenet { 6802.25, 2.88392 }, Point { 646.659, 1169.14 }), make_tuple(
          Frenet { 6806.4, 2.88474 }, Point { 650.275, 1167.11 }), make_tuple(
          Frenet { 6810.13, 2.83917 }, Point { 653.552, 1165.32 }), make_tuple(
          Frenet { 6814.13, 2.74098 }, Point { 657.086, 1163.45 }), make_tuple(
          Frenet { 6817.95, 2.60227 }, Point { 660.48, 1161.7 }), make_tuple(
          Frenet { 6821.93, 2.40694 }, Point { 664.051, 1159.92 }), make_tuple(
          Frenet { 6825.34, 2.20527 }, Point { 667.117, 1158.43 }), make_tuple(
          Frenet { 6828.24, 2.02575 }, Point { 670.015, 1157.06 }), make_tuple(
          Frenet { 6831.55, 2.25616 }, Point { 673.023, 1155.66 }), make_tuple(
          Frenet { 6834.65, 2.44951 }, Point { 675.852, 1154.37 }), make_tuple(
          Frenet { 6837.56, 2.60958 }, Point { 678.515, 1153.18 }), make_tuple(
          Frenet { 6840.4, 2.74197 }, Point { 681.125, 1152.05 }), make_tuple(
          Frenet { 6843.18, 2.84227 }, Point { 683.683, 1150.96 }), make_tuple(
          Frenet { 6846.12, 2.91593 }, Point { 686.405, 1149.85 }), make_tuple(
          Frenet { 6849.55, 2.96536 }, Point { 689.593, 1148.58 }), make_tuple(
          Frenet { 6852.5, 2.96843 }, Point { 692.342, 1147.52 }), make_tuple(
          Frenet { 6855.93, 2.92475 }, Point { 695.563, 1146.34 }), make_tuple(
          Frenet { 6859.18, 2.83434 }, Point { 698.63, 1145.27 }), make_tuple(
          Frenet { 6863.32, 2.6522 }, Point { 702.567, 1143.96 }), make_tuple(
          Frenet { 6867.28, 2.39791 }, Point { 706.352, 1142.78 }), make_tuple(
          Frenet { 6870.34, 2.148 }, Point { 709.306, 1141.92 }), make_tuple(
          Frenet { 6873.47, 2.2131 }, Point { 712.728, 1141 }), make_tuple(
          Frenet { 6876.2, 2.42302 }, Point { 715.385, 1140.34 }), make_tuple(
          Frenet { 6878.79, 2.57947 }, Point { 717.914, 1139.75 }), make_tuple(
          Frenet { 6881.51, 2.70579 }, Point { 720.576, 1139.17 }), make_tuple(
          Frenet { 6884.33, 2.79739 }, Point { 723.335, 1138.62 }), make_tuple(
          Frenet { 6886.2, 2.83997 }, Point { 725.178, 1138.26 }), make_tuple(
          Frenet { 6888.21, 2.8692 }, Point { 727.149, 1137.9 }), make_tuple(
          Frenet { 6890.73, 2.88313 }, Point { 729.638, 1137.47 }), make_tuple(
          Frenet { 6893.5, 2.8678 }, Point { 732.364, 1137.02 }), make_tuple(
          Frenet { 6896.47, 2.81504 }, Point { 735.309, 1136.58 }), make_tuple(
          Frenet { 6899.7, 2.72321 }, Point { 738.505, 1136.13 }), make_tuple(
          Frenet { 6903.15, 2.58881 }, Point { 741.935, 1135.69 }), };

  for (const tuple<Frenet, Point>& frenetPointTuple : frenetPointTuples) {
    const Frenet& frenet = get<0>(frenetPointTuple);
    const Point& point = get<1>(frenetPointTuple);
    test_convert(point, frenet);
  }
}

#endif
