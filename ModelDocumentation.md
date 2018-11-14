# Model Documentation

TODO:
- detail how the project was completed.
- A great write-up should include the rubric points as well as your description of how you addressed each point. You should include a detailed description of the code used in each step (with line-number references and code snippets where appropriate) and links to other supporting documents or external references.
- just a brief description of how you passed each rubric point, and references to the relevant code :)
- The code model for generating paths is described in detail.

## Path Planning

Pfaderzeugung (`PathPlanner::createPath()`):
Falls ein anderes Fahrzeug im Weg ist (`isAnyVehicleWithin30MetersAheadOfEgoCarInLane()`), bremse ([slowDown()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/c1bd39ef43180098045da42c8c21684c68ab09db/src/pathPlanner.h#L189)) und wechsle wenn möglich auf eine freie Fahrbahn (`getNewLane()`). Bleibe ansonsten auf der aktuellen Fahrbahn und beschleunige ([speedUp()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/c1bd39ef43180098045da42c8c21684c68ab09db/src/pathPlanner.h#L195)) so lange bis die Maximalgeschwindigkeit von 50 mph erreicht ist.

The `PathPlanner`'s [createPath()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/fb67df3bc224322f1845d7bdadd1c9d4518d7af9/src/pathPlanner.h#L89) method creates a path by planning the ego car's behaviour (`planBehaviour()`) and generating a path (`generatePath()`):

```
pathPlanner.h:
tuple<Path, Lane, ReferencePoint> PathPlanner::createPath() {
  Lane newLane;
  ReferencePoint refPointNew;
  tie(newLane, refPointNew) = planBehaviour();
  Path path;
  tie(path, refPointNew) = generatePath(refPointNew);
  return make_tuple(path, newLane, refPointNew);
}
```

### Behavior Planning

- `planBehaviour()` first checks whether any vehicle is too close to the ego car i.e. whether any vehicle is within 30 meters ahead of the ego car in lane. If that is the case a new 'free' (in welchem Sinne free? siehe PathPlanner::canSwitch2Lane()) lane is searched for. Additionally the ego car's new velocity is computed (`getNewVelocityMph()`).
Ergebnis: newLane und neue Geschwindigkeit refPointNew.vel_mph.

```
pathPlanner.h:
tuple<Lane, ReferencePoint> PathPlanner::planBehaviour() {
  bool tooClose = isAnyVehicleWithin30MetersAheadOfEgoCarInLane(lane);
  Lane newLane = getNewLane(tooClose, lane);
  ReferencePoint refPointNew;
  refPointNew.vel_mph = getNewVelocityMph(tooClose, refPoint.vel_mph);
  refPointNew.point = egoCarAtEndOfPath.getPos();
  refPointNew.yaw_rad = deg2rad(egoCarAtEndOfPath.yaw_deg);
  return make_tuple(newLane, refPointNew);
}
```

### Trajectory Generation

PathCreator.h genauer erklären:

```
pathCreator.h:
  vector<FrenetCart> createNewPoints(const Lane& lane) {
    auto egoCarPlus =
        [&](int s_offset) {
          return createFrenetCart(
              Frenet {egoCar.getPos().getFrenet().s + s_offset, getMiddleOfLane(lane)});
        };

    return {egoCarPlus(30), egoCarPlus(60), egoCarPlus(90)};
  }
```

## The car is able to drive at least 4.32 miles without incident

## The car drives according to the speed limit

```
pathPlanner.h:
 1: double PathPlanner::getNewVelocityMph(const bool tooClose,
 2:                                       const double actualVelMph) {
 3:   auto slowDown = [&]() {
 4:     const double newVelMph = actualVelMph - 0.75;
 5:     return newVelMph > 0 ? newVelMph : actualVelMph;
 6:   };
 7: 
 8:  auto speedUp = [&]() {
 9:     const double newVelMph = actualVelMph + 0.25;
10:     return newVelMph < speed_limit_mph ? newVelMph : speed_limit_mph;
11:   };
12: 
13:   return tooClose ? slowDown() : speedUp();
14: }
```

## Max Acceleration and Jerk are not Exceeded

dt = 0.02.
berechne aus obiger Methode getNewVelocityMph() die ungefähre Beschleunigung und zeige sie ist <= 10 m/s^2.

## Car does not have collisions

## The car stays in its lane, except for the time between changing lanes

## The car is able to change lanes
```
pathPlanner.h:
Lane PathPlanner::getNewLane(bool tooClose, const Lane& actualLane) {
  if (!tooClose) {
    return actualLane;
  }

  auto canSwitchFromLaneToLane = [&](const Lane& from, const Lane& to) {
    return actualLane == from && canSwitch2Lane(to);
  };

  if (canSwitchFromLaneToLane(Lane::LEFT, Lane::MIDDLE)) {
    return Lane::MIDDLE;
  }

  if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::LEFT)
      && canSwitchFromLaneToLane(Lane::MIDDLE, Lane::RIGHT)) {

    return getMoreFreeLeftOrRightLane();
  }

  if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::LEFT)) {
    return Lane::LEFT;
  }

  if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::RIGHT)) {
    return Lane::RIGHT;
  }

  if (canSwitchFromLaneToLane(Lane::RIGHT, Lane::MIDDLE)) {
    return Lane::MIDDLE;
  }

  return actualLane;
}

bool PathPlanner::canSwitch2Lane(const Lane& lane) {
  return !isAnyVehicleWithin30MetersAheadOfEgoCarInLane(lane)
      && !isAnyVehicleInLaneBehindOfEgoCarInTheWay(lane);
}
```

