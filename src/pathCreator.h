#ifndef PATHCREATOR_H_
#define PATHCREATOR_H_

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>
#include <tuple>

#include "coords/coordinateSystem.h"
#include "coords/coordsConverter.h"
#include "coords/frenet.h"
#include "coords/frenetCart.h"
#include "egoCar.h"
#include "funs.h"
#include "lane.h"
#include "path.h"
#include "previousData.h"
#include "referencePoint.h"
#include "spline.h"

using namespace std;

class PathCreator {

 public:
  PathCreator(const CoordsConverter& _coordsConverter,
              const PreviousData& _previousData, const EgoCar& _egoCar,
              double _dt, const Lane& _lane)
      : coordsConverter(_coordsConverter),
        previousData(_previousData),
        egoCar(_egoCar),
        dt(_dt),
        lane(_lane) {
  }

  Path createPath(ReferencePoint& refPoint) {
    Path path;
    appendSnd2Fst(path.points, previousData.previous_path.points);
    appendSnd2Fst(path.points, createNewPathPoints(refPoint));
    return path;
  }

 private:
  vector<FrenetCart> createNewPathPoints(ReferencePoint& refPoint) {
    Path path;
    ReferencePoint refPointNew = addPointsFromPreviousData(path, refPoint);
    refPoint = refPointNew;
    addNewPoints(path);
    return createSplinePoints(path, refPoint);
  }

  ReferencePoint addPointsFromPreviousData(Path& path,
                                           const ReferencePoint& refPoint) {
    vector<FrenetCart> points;
    ReferencePoint refPointNew;
    tie(points, refPointNew) = createPointsFromPreviousData(refPoint);
    appendSnd2Fst(path.points, points);
    return refPointNew;
  }

  void addNewPoints(Path& path) {
    appendSnd2Fst(path.points, createNewPoints());
  }

  tuple<vector<FrenetCart>, ReferencePoint> createPointsFromPreviousData(
      const ReferencePoint& refPoint) {
    vector<FrenetCart> points;
    ReferencePoint refPointNew = refPoint;

    if (previousData.sizeOfPreviousPath() < 2) {
      Frenet prev = egoCar.getPos().getFrenet()
          - Frenet::fromAngle(deg2rad(egoCar.yaw_deg));

      points.push_back(createFrenetCart(prev));
      points.push_back(createFrenetCart(egoCar.getPos().getFrenet()));
    } else {
      refPointNew.point = previousData.previous_path.points[previousData
          .sizeOfPreviousPath() - 1].getFrenet();
      Frenet prev = previousData.previous_path.points[previousData
          .sizeOfPreviousPath() - 2].getFrenet();
      refPointNew.yaw_rad = (refPointNew.point - prev).getHeading();

      points.push_back(createFrenetCart(prev));
      points.push_back(createFrenetCart(refPointNew.point));
    }

    return make_tuple(points, refPointNew);
  }

  vector<FrenetCart> createSplinePoints(const Path& path,
                                        const ReferencePoint& refPoint) {
    return workWithPathInCarsCoordinateSystem(
        path,
        [&](const Path& carsPath) {
          return createSplinePoints(carsPath.asSpline(), path_size - previousData.sizeOfPreviousPath(), refPoint);
        },
        refPoint);
  }

  vector<FrenetCart> workWithPathInCarsCoordinateSystem(
      const Path& path,
      const function<vector<FrenetCart>(const Path& carsPath)>& transformCarsPath2Points,
      const ReferencePoint& refPoint) {

    Path carsPath;
    carsPath.points = enterCarsCoordinateSystem(refPoint.point,
                                                -refPoint.yaw_rad, path.points);
    sort_and_remove_duplicates(carsPath.points);
    vector<FrenetCart> points = transformCarsPath2Points(carsPath);
    return leaveCarsCoordinateSystem(refPoint.point, refPoint.yaw_rad, points);
  }

  vector<FrenetCart> enterCarsCoordinateSystem(
      const Frenet& origin, const double angle_rad,
      const vector<FrenetCart>& points) {

    CoordinateSystem coordinateSystem = createRotatedCoordinateSystem(
        Frenet::zero(), angle_rad);
    vector<FrenetCart> origin2points = map2<FrenetCart, FrenetCart>(
        points, [&](const FrenetCart& point) {
          return createFrenetCart(point.getFrenet() - origin);
        });
    return transform(coordinateSystem, origin2points);
  }

  vector<FrenetCart> leaveCarsCoordinateSystem(
      const Frenet& origin, double angle_rad,
      const vector<FrenetCart>& points) {

    return transform(createRotatedCoordinateSystem(origin, angle_rad), points);
  }

  vector<FrenetCart> transform(const CoordinateSystem& coordinateSystem,
                               const vector<FrenetCart>& points) const {

    return map2<FrenetCart, FrenetCart>(points, [&](const FrenetCart& point) {
      return createFrenetCart(coordinateSystem.transform(point.getFrenet()));});
  }

  CoordinateSystem createRotatedCoordinateSystem(const Frenet& origin,
                                                 double angle_rad) {
    Frenet e1 = Frenet { cos(angle_rad), sin(angle_rad) };
    Frenet e2 = Frenet { -sin(angle_rad), cos(angle_rad) };
    return CoordinateSystem { origin, e1, e2 };
  }

  FrenetCart createFrenetCart(Frenet frenet) const {
    return FrenetCart(frenet, coordsConverter);
  }

  vector<FrenetCart> createNewPoints() {
    auto egoCarPlus =
        [&](int s_offset) {
          return createFrenetCart(
              Frenet {egoCar.getPos().getFrenet().s + s_offset, getMiddleOfLane(lane)});
        };

    return {egoCarPlus(30), egoCarPlus(60), egoCarPlus(90)};
  }

  vector<FrenetCart> createSplinePoints(const Spline& spline, const int num,
                                        const ReferencePoint& refPoint) {

    vector<double> s_vals = createSVals(spline, num, refPoint);
    vector<FrenetCart> points = map2<double, FrenetCart>(
        s_vals,
        [&](const double s_val) {return createSplinePoint(s_val, spline);});
    return points;
  }

  // TODO: hier sollen s_vals erzeugt werden, die einen Abstand nach der Bogenlänge s_delta der Splinekurve spline haben.
  vector<double> createSVals(const Spline& spline, const int num,
                             const ReferencePoint& refPoint) {
    vector<double> s_vals;
    const double s_delta = dt * mph2meter_per_sec(0.89 * refPoint.vel_mph);
    for (int i = 0; i < num; i++) {
      s_vals.push_back((i + 1) * s_delta);
    }
    return s_vals;
  }

  FrenetCart createSplinePoint(double x, const Spline& spline) {
    return createFrenetCart(Frenet { x, spline(x) });
  }

  void sort_and_remove_duplicates(vector<FrenetCart>& points) {
    std::sort(
        points.begin(),
        points.end(),
        [&](const FrenetCart& p1, const FrenetCart& p2) {return p1.getFrenet().s < p2.getFrenet().s;});
    points.erase(
        unique(
            points.begin(),
            points.end(),
            [&](const FrenetCart& p1, const FrenetCart& p2) {return p1.getFrenet().s == p2.getFrenet().s;}),
        points.end());
  }

  const CoordsConverter& coordsConverter;
  const PreviousData& previousData;
  const EgoCar& egoCar;
  const int path_size = 50;
  const double dt;
  const Lane& lane;
};

#endif /* PATHCREATOR_H_ */
