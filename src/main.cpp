#include "pathPlanner.h"
#include "lane.h"
#include "gtest/gtest.h"

#include "coordsConverterTest.cpp"
#include "tests.cpp"

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

EgoCar createEgoCar(
    const nlohmann::basic_json<std::map, std::vector,
        std::__cxx11::basic_string<char, std::char_traits<char>,
            std::allocator<char> >, bool, long, unsigned long, double,
        std::allocator, nlohmann::adl_serializer> &j,
    const CoordsConverter& coordsConverter) {
  EgoCar egoCar(coordsConverter);
  egoCar.setPos(Point { j[1]["x"], j[1]["y"] },
                Frenet { j[1]["s"], j[1]["d"] });
  egoCar.yaw_deg = j[1]["yaw"];
  egoCar.speed_mph = j[1]["speed"];
  return egoCar;
}

// Sensor Fusion Data, a list of all other cars on the same side of the road.
vector<Vehicle> createVehicles(
    const nlohmann::basic_json<std::map, std::vector,
        std::__cxx11::basic_string<char, std::char_traits<char>,
            std::allocator<char> >, bool, long, unsigned long, double,
        std::allocator, nlohmann::adl_serializer> &sensor_fusion,
    const CoordsConverter& coordsConverter) {
  enum sensor_fusion_index {
    ID = 0,
    X = 1,
    Y = 2,
    VX = 3,
    VY = 4,
    S = 5,
    D = 6
  };

  vector<Vehicle> vehicles;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    Vehicle vehicle(coordsConverter);
    vehicle.id = sensor_fusion[i][ID];
    vehicle.setPos(Point { sensor_fusion[i][X], sensor_fusion[i][Y] }, Frenet {
                       sensor_fusion[i][S], sensor_fusion[i][D] });
    vehicle.setVel_cart_m_per_s(Point { sensor_fusion[i][VX],
        sensor_fusion[i][VY] });
    vehicles.push_back(vehicle);
  }
  return vehicles;
}

int main(int argc, char **argv) {
  if (argc > 1 && strcmp(argv[1], "test") == 0) {
    testing::InitGoogleTest(&argc, argv);
    // see https://stackoverflow.com/questions/7208070/googletest-how-to-skip-a-test
    // testing::GTEST_FLAG(filter) = "-PathPlanningTest.should_drive_with_max_50_mph";
    // testing::GTEST_FLAG(filter) = "CoordsConverterTest.*";
    return RUN_ALL_TESTS();
  }

  uWS::Hub h;

  const MapWaypoints mapWaypoints = read_map_waypoints();
  const CoordsConverter coordsConverter(mapWaypoints);

  Lane lane = Lane::MIDDLE;
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  double dt = 0.02;

  h.onMessage(
      [dt, &refPoint,&lane,&coordsConverter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
          uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
          auto s = hasData(data);

          if (s != "") {
            nlohmann::basic_json<std::map, std::vector, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, bool, long, unsigned long, double, std::allocator, nlohmann::adl_serializer> j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry") {
              // j[1] is the data JSON object

              EgoCar egoCar = createEgoCar(j, coordsConverter);
              Path next_vals = createPath(refPoint,
                  lane,
                  coordsConverter,
                  egoCar,
                  createPreviousData(j),
                  createVehicles(j[1]["sensor_fusion"], coordsConverter),
                  dt);

              json msgJson;
              vector<double> xs;
              vector<double> ys;
              tie(xs, ys) = getPoints(next_vals);
              msgJson["next_x"] = xs;
              msgJson["next_y"] = ys;

              auto msg = "42[\"control\","+ msgJson.dump()+"]";

              //this_thread::sleep_for(chrono::milliseconds(1000));
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          } else {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
