# Model Documentation


## Path Planning

A path for the ego car to follow is created ([PathPlanner::createPath()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/43f9a2e1f6289d8c01256a43bf1e2411dbe2ed4c/src/pathPlanner.h#L89)) using the following simple strategy:

Stay in the current lane and [speed up](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/c1bd39ef43180098045da42c8c21684c68ab09db/src/pathPlanner.h#L195) until the maximum speed of 50 mph is reached. However, if another vehicle gets in the way ([isAnyVehicleWithin30MetersAheadOfEgoCarInLane()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/43f9a2e1f6289d8c01256a43bf1e2411dbe2ed4c/src/pathPlanner.h#L119)), [slow down](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/c1bd39ef43180098045da42c8c21684c68ab09db/src/pathPlanner.h#L189) and change to a free lane if possible ([getNewLane()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/43f9a2e1f6289d8c01256a43bf1e2411dbe2ed4c/src/pathPlanner.h#L258)).


## The car drives according to the speed limit

The method [getNewVelocityMph()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/5c5a44cc69410de5ac237e2de1051dd33def927e/src/pathPlanner.h#L187) calculates the desired speed of the ego car:

```
 1: double PathPlanner::getNewVelocityMph(const bool tooClose,
 2:                                       const double actualVelMph) {
 3:   auto slowDown = [&]() {
 4:     const double newVelMph = actualVelMph - 0.75;
 5:     return newVelMph > 0 ? newVelMph : actualVelMph;
 6:   };
 7:
 8:  auto speedUp = [&]() {
 9:     const double newVelMph = actualVelMph + 0.4;
10:     return newVelMph < speed_limit_mph ? newVelMph : speed_limit_mph;
11:   };
12:
13:   return tooClose ? slowDown() : speedUp();
14: }
```

The new velocity is alway between 0 (see line 5) and `speed_limit_mph` = 50 mph (see line 10), so the car drives according to the speed limit.

## Max Acceleration is not Exceeded

Acceleration uses the formula: [newVelMph = actualVelMph + 0.4](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/5c5a44cc69410de5ac237e2de1051dd33def927e/src/pathPlanner.h#L196).
So the change $dv$ in velocity is:
$dv = \textit{newVelMph} - \textit{actualVelMph} = 0.4 \textit{ mph} = \frac{0.4}{2.24} \frac{m}{s} = 0.178571429 \frac{m}{s}$. Given that $dt = 0.02 s$ the acceleration $a$ is:
$a = \frac{dv}{dt} = \frac{0.178571429}{0.02} \frac{m}{s^2} = 8.928571429 \frac{m}{s^2}$, which does not exceed the max acceleration of $10 \frac{m}{s^2}$.

## Car does not have collisions

The path planner tries to avoid collisions by maintaining a minimum distance of 30 meters to other vehicles. Before switching lanes the path planner checks in the method [canSwitch2Lane()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/1211cebcdf2907b05ab3493c58e22481d9719ed8/src/pathPlanner.h#L151) whether a lane switch is safe by assuring that no vehicle in the desired lane is in the way neither in front of the ego car (`!isAnyVehicleWithin30MetersAheadOfEgoCarInLane(lane)`) nor behind the ego car (`!isAnyVehicleInLaneBehindOfEgoCarInTheWay(lane)`):

```
bool PathPlanner::canSwitch2Lane(const Lane& lane) {
  return !isAnyVehicleWithin30MetersAheadOfEgoCarInLane(lane)
      && !isAnyVehicleInLaneBehindOfEgoCarInTheWay(lane);
}
```

## The car stays in its lane

Whenever the car is not too close to any other vehicle then the method ([getNewLane()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/43f9a2e1f6289d8c01256a43bf1e2411dbe2ed4c/src/pathPlanner.h#L258)) returns the actual lane:

```
  if (!tooClose) {
    return actualLane;
  }
```

So the car stays in its lane.

## The car is able to change lanes

The method [getNewLane()](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/43f9a2e1f6289d8c01256a43bf1e2411dbe2ed4c/src/pathPlanner.h#L258) returns the adjacent lane to which the ego car can switch from its actual lane:

```
 1: Lane PathPlanner::getNewLane(bool tooClose, const Lane& actualLane) {
 2:   if (!tooClose) {
 3:     return actualLane;
 4:   }
 5: 
 6:   auto canSwitchFromLaneToLane = [&](const Lane& from, const Lane& to) {
 7:     return actualLane == from && canSwitch2Lane(to);
 8:   };
 9: 
10:   if (canSwitchFromLaneToLane(Lane::LEFT, Lane::MIDDLE)) {
11:     return Lane::MIDDLE;
12:   }
13: 
14:   if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::LEFT)
15:       && canSwitchFromLaneToLane(Lane::MIDDLE, Lane::RIGHT)) {
16: 
17:     return getMoreFreeLeftOrRightLane();
18:   }
19: 
20:   if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::LEFT)) {
21:     return Lane::LEFT;
22:   }
23: 
24:   if (canSwitchFromLaneToLane(Lane::MIDDLE, Lane::RIGHT)) {
25:     return Lane::RIGHT;
26:   }
27: 
28:   if (canSwitchFromLaneToLane(Lane::RIGHT, Lane::MIDDLE)) {
29:     return Lane::MIDDLE;
30:   }
31: 
32:   return actualLane;
33: }

```

When a lane switch from the middle lane to both the
left and right lane is possible then the lane which is *more free* is preferred (see lines 14 - 18).

TODO: Bibliotheken nennen:
- alglib für kubische Splines.
- boost für ...
