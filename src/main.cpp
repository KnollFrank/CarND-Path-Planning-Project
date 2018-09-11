#include "pathPlanner.h"
#include "gtest/gtest.h"
#include "tests.cpp"

Frenet Frenet::operator+(const Frenet &other) const {
  Frenet sum;
  sum.s = s + other.s;
  sum.d = d + other.d;
  return sum;
}

Frenet Frenet::operator-(const Frenet &other) const {
  return *this + (other * -1);
}

Frenet Frenet::operator*(double scalar) const {
  Frenet result;
  result.s = s * scalar;
  result.d = d * scalar;
  return result;
}

Point Point::fromAngle(double angle_rad) {
  return Point { cos(angle_rad), sin(angle_rad) };
}

double Point::len() const {
  return distanceTo(Point { 0, 0 });
}

double Point::scalarProd(const Point &point) const {
  return x * point.x + y * point.y;
}

double Point::distanceTo(const Point &point) const {
  Point diff = point - *this;
  return sqrt(diff.scalarProd(diff));
}

double Point::getHeading() const {
  return atan2(y, x);
}

Point Point::operator+(const Point &other) const {
  Point sum;
  sum.x = x + other.x;
  sum.y = y + other.y;
  return sum;
}

Point Point::operator-(const Point &other) const {
  return *this + (other * -1);
}

Point Point::operator*(double scalar) const {
  Point result;
  result.x = x * scalar;
  result.y = y * scalar;
  return result;
}

void EgoCar::setPos(const Point &pos_cart, const Frenet &pos_frenet) {
  this->pos_cart = pos_cart;
  this->pos_frenet = pos_frenet;
}

void EgoCar::setPos_cart(const Point &pos, const MapWaypoints &map_waypoints) {
  pos_cart = pos;
  pos_frenet = getFrenet(pos, 0, map_waypoints);
}

Point EgoCar::getPos_cart() const {
  return pos_cart;
}

void EgoCar::setPos_frenet(const Frenet &pos,
                           const MapWaypoints &map_waypoints) {
  pos_frenet = pos;
  pos_cart = getXY(pos, map_waypoints);
}

Frenet EgoCar::getPos_frenet() const {
  return pos_frenet;
}

void Vehicle::setPos(const Point &pos_cart, const Frenet &pos_frenet) {
  this->pos_cart = pos_cart;
  this->pos_frenet = pos_frenet;
}

void Vehicle::setPos_cart(const Point &pos, const MapWaypoints &map_waypoints) {
  pos_cart = pos;
  pos_frenet = getFrenet(pos, 0, map_waypoints);
}

Point Vehicle::getPos_cart() const {
  return pos_cart;
}

void Vehicle::setPos_frenet(const Frenet &pos,
                            const MapWaypoints &map_waypoints) {
  pos_frenet = pos;
  pos_cart = getXY(pos, map_waypoints);
}

Frenet Vehicle::getPos_frenet() const {
  return pos_frenet;
}

void Vehicle::setVel_cart_m_per_s(const Point &vel) {
  this->vel_cart_m_per_s = vel;
}

Point Vehicle::getVel_cart_m_per_s() const {
  return vel_cart_m_per_s;
}

void Vehicle::setVel_frenet_m_per_s(const Frenet &vel,
                                    const MapWaypoints &map_waypoints) {
  const Frenet &src = getPos_frenet();
  const Frenet &dst = Frenet { src.s + vel.s, src.d + vel.d };
  vel_cart_m_per_s = createCartVectorConnectingStartAndEnd(src, dst,
                                                           map_waypoints);
}

Frenet Vehicle::getVel_frenet_m_per_s(const MapWaypoints &map_waypoints) const {
  const Point &src = getPos_cart();
  const Point &dst = src + getVel_cart_m_per_s();
  return createFrenetVectorConnectingStartAndEnd(src, dst, map_waypoints);
}

int main(int argc, char **argv) {
  if (argc > 1 && strcmp(argv[1], "test") == 0) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  }

  uWS::Hub h;

  MapWaypoints map_waypoints = read_map_waypoints();

  // TODO: make lane an enum: LEFT=0, MIDDLE=1, RIGHT=2
  int lane = 1;
  ReferencePoint refPoint;
  refPoint.vel_mph = 0;
  double dt = 0.02;

  h.onMessage(
      [dt, &refPoint,&lane,&map_waypoints](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

              EgoCar egoCar = createEgoCar(j);
              Path next_vals = createPath(refPoint, lane, map_waypoints, egoCar, createPreviousData(j), createVehicles(j[1]["sensor_fusion"]), dt);

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
