#ifndef PATHCREATOR_H_
#define PATHCREATOR_H_

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>
#include <tuple>

#include "coords/coordinateSystemCart.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "egoCar.h"
#include "funs.h"
#include "lane.h"
#include "path.h"
#include "referencePoint.h"
#include "spline.h"

using namespace std;

class PathCreator {

 public:
  PathCreator(const CoordsConverter& _coordsConverter, const EgoCar& _egoCar,
              double _dt, const ReferencePoint& _refPoint)
      : coordsConverter(_coordsConverter),
        egoCar(_egoCar),
        dt(_dt),
        refPoint(_refPoint) {
  }

  tuple<Path, ReferencePoint> createPath(const Path& previousPath,
                                         const Lane& lane) {
    Path path;
    appendSnd2Fst(path.points, previousPath.points);
    vector<FrenetCart> points;
    ReferencePoint refPointNew;
    tie(points, refPointNew) = createNewPathPoints(previousPath, lane);
    appendSnd2Fst(path.points, points);
    return make_tuple(path, refPointNew);
  }

 private:
  tuple<vector<FrenetCart>, ReferencePoint> createNewPathPoints(
      const Path& previousPath, const Lane& lane) {
    Path path;
    const ReferencePoint refPointNew = addPointsFromPreviousPath(path,
                                                                 previousPath);
    addNewPoints(path, lane);
    vector<FrenetCart> splinePoints = createSplinePoints(path, refPointNew,
                                                         previousPath);
    return make_tuple(splinePoints, refPointNew);
  }

  ReferencePoint addPointsFromPreviousPath(Path& path,
                                           const Path& previousPath) {
    vector<FrenetCart> points;
    ReferencePoint refPointNew;
    tie(points, refPointNew) = createPointsFromPreviousPath(previousPath);
    appendSnd2Fst(path.points, points);
    return refPointNew;
  }

  void addNewPoints(Path& path, const Lane& lane) {
    appendSnd2Fst(path.points, createNewPoints(lane));
  }

  tuple<vector<FrenetCart>, ReferencePoint> createPointsFromPreviousPath(
      const Path& previousPath) {
    vector<FrenetCart> points;
    ReferencePoint refPointNew = refPoint;

    if (previousPath.points.size() < 2) {
      Point prev = egoCar.getPos().getXY()
          - Point::fromAngle(deg2rad(egoCar.yaw_deg));

      points.push_back(createFrenetCart(prev));
      points.push_back(egoCar.getPos());
    } else {
      refPointNew.point = previousPath.points[previousPath.points.size() - 1];
      FrenetCart prev = previousPath.points[previousPath.points.size() - 2];
      refPointNew.yaw_rad = (refPointNew.point.getXY() - prev.getXY())
          .getHeading();

      points.push_back(prev);
      points.push_back(refPointNew.point);
    }

    return make_tuple(points, refPointNew);
  }

  vector<FrenetCart> createSplinePoints(const Path& path,
                                        const ReferencePoint& refPoint,
                                        const Path& previousPath) {
    return workWithPathInCarsCoordinateSystem(
        path,
        [&](const Path& carsPath) {
          return createSplinePoints(carsPath.asXYSpline(), path_size - previousPath.points.size(), refPoint);
        },
        refPoint);
  }

  vector<FrenetCart> workWithPathInCarsCoordinateSystem(
      const Path& path,
      const function<vector<FrenetCart>(const Path& carsPath)>& transformCarsPath2Points,
      const ReferencePoint& refPoint) {

    Path carsPath;
    // FIXME: nach enterCarsCoordinateSystem macht es nur noch Sinn mit Frenet-Koordinaten weiterzurechnen innerhalb von
    // transformCarsPath2Points, denn die aus FrenetCart gewonnenen XY-Koordinaten sind ab hier FALSCH. Also Signaturen der abhängigen Methoden von FrenetCart ändern in Frenet.
    // FrenetCart im Rückgabewert von workWithPathInCarsCoordinateSystem ist aber wieder ok.
    carsPath.points = enterCarsCoordinateSystem(refPoint.point.getXY(),
                                                -refPoint.yaw_rad, path.points);
    remove_duplicates(carsPath.points);
    vector<FrenetCart> points = transformCarsPath2Points(carsPath);
    return leaveCarsCoordinateSystem(refPoint.point.getXY(), refPoint.yaw_rad,
                                     points);
  }

  vector<FrenetCart> enterCarsCoordinateSystem(
      const Point& origin, const double angle_rad,
      const vector<FrenetCart>& points) {

    CoordinateSystemCart coordinateSystem = createRotatedCoordinateSystem(
        Point::zero(), angle_rad);
    vector<FrenetCart> origin2points = map2<FrenetCart, FrenetCart>(
        points, [&](const FrenetCart& point) {
          return createFrenetCart(point.getXY() - origin);
        });
    return transform(coordinateSystem, origin2points);
  }

  vector<FrenetCart> leaveCarsCoordinateSystem(
      const Point& origin, double angle_rad, const vector<FrenetCart>& points) {

    return transform(createRotatedCoordinateSystem(origin, angle_rad), points);
  }

  vector<FrenetCart> transform(const CoordinateSystemCart& coordinateSystem,
                               const vector<FrenetCart>& points) const {

    return map2<FrenetCart, FrenetCart>(points, [&](const FrenetCart& point) {
      return createFrenetCart(coordinateSystem.transform(point.getXY()));});
  }

  CoordinateSystemCart createRotatedCoordinateSystem(const Point& origin,
                                                     double angle_rad) {
    Point e1 = Point { cos(angle_rad), sin(angle_rad) };
    Point e2 = Point { -sin(angle_rad), cos(angle_rad) };
    return CoordinateSystemCart { origin, e1, e2 };
  }

  FrenetCart createFrenetCart(const Frenet& frenet) const {
    return FrenetCart(frenet, coordsConverter);
  }

  FrenetCart createFrenetCart(const Point& point) const {
    return FrenetCart(point, coordsConverter);
  }

  vector<FrenetCart> createNewPoints(const Lane& lane) {
    auto egoCarPlus =
        [&](int s_offset) {
          return createFrenetCart(
              Frenet {egoCar.getPos().getFrenet().s + s_offset, getMiddleOfLane(lane)});
        };

    return {egoCarPlus(30), egoCarPlus(60), egoCarPlus(90)};
  }

  vector<FrenetCart> createSplinePoints(const Spline& spline, const int num,
                                        const ReferencePoint& refPoint) {

    vector<double> s_vals = createSVals(spline, num, refPoint.vel_mph);
    vector<FrenetCart> points = map2<double, FrenetCart>(
        s_vals,
        [&](const double s_val) {return createSplinePoint(s_val, spline);});
    return points;
  }

  vector<double> createSVals(const Spline& spline, const int num,
                             double vel_mph) {
    vector<double> s_vals;
    const double s_delta = dt * mph2meter_per_sec(vel_mph);
    for (int i = 0; i < num; i++) {
      s_vals.push_back((i + 1) * s_delta);
    }
    return s_vals;
  }

  FrenetCart createSplinePoint(double x, const Spline& spline) {
    return createFrenetCart(Point { x, spline(x) });
  }

  void remove_duplicates(vector<FrenetCart>& points) {
    points.erase(
        unique(
            points.begin(),
            points.end(),
            [&](const FrenetCart& p1, const FrenetCart& p2) {return p1.getFrenet().s == p2.getFrenet().s;}),
        points.end());
  }

  const CoordsConverter& coordsConverter;
  const EgoCar& egoCar;
  const int path_size = 50;
  const double dt;
  const ReferencePoint& refPoint;
};

#endif /* PATHCREATOR_H_ */
