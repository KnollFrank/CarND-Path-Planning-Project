# Model Documentation

TODO:
- detail how the project was completed.
- A great write-up should include the rubric points as well as your description of how you addressed each point. You should include a detailed description of the code used in each step (with line-number references and code snippets where appropriate) and links to other supporting documents or external references.
- just a brief description of how you passed each rubric point, and references to the relevant code :)
- The code model for generating paths is described in detail.

## Path Planning

A path for the ego car to follow is created  (`PathPlanner::createPath()`) according to the following simple strategy:

Stay in the current lane and [speed up](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/c1bd39ef43180098045da42c8c21684c68ab09db/src/pathPlanner.h#L195) until the maximum speed of 50 mph is reached. However, if another vehicle gets in the way (`isAnyVehicleWithin30MetersAheadOfEgoCarInLane()`), [slow down](https://github.com/KnollFrank/CarND-Path-Planning-Project/blob/c1bd39ef43180098045da42c8c21684c68ab09db/src/pathPlanner.h#L189) and change to a free lane if possible (`getNewLane()`).


## The car drives according to the speed limit

The following method calculates the desired speed of the ego car:

```
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

The new velocity is alway between 0 (see line 5) and `speed_limit_mph` = 50 mph (see line 10).

## Max Acceleration is not Exceeded

dt = 0.02.
berechne aus obiger Methode getNewVelocityMph() die ungefähre Beschleunigung und zeige sie ist <= 10 m/s^2.

## Car does not have collisions

Durch den Sicherheitsabstand von 30 Metern und den Test auf freie Bahn sollten Kollisionen mit anderen Fahrzeugen vermieden werden.

```
bool PathPlanner::canSwitch2Lane(const Lane& lane) {
  return !isAnyVehicleWithin30MetersAheadOfEgoCarInLane(lane)
      && !isAnyVehicleInLaneBehindOfEgoCarInTheWay(lane);
}
```

## The car stays in its lane
siehe getNewLane():

```
  if (!tooClose) {
    return actualLane;
  }
```

## The car is able to change lanes
```
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

```

