# Model Documentation

TODO:
- detail how the project was completed.
- A great write-up should include the rubric points as well as your description of how you addressed each point. You should include a detailed description of the code used in each step (with line-number references and code snippets where appropriate) and links to other supporting documents or external references.
- just a brief description of how you passed each rubric point, and references to the relevant code :)
- The code model for generating paths is described in detail.

## The car drives according to the speed limit

## Max Acceleration and Jerk are not Exceeded

## Car does not have collisions

## The car stays in its lane, except for the time between changing lanes

## The car is able to change lanes

```
tuple<Path, Lane, ReferencePoint> PathPlanner::createPath() {
  Lane newLane;
  ReferencePoint refPointNew;
  tie(newLane, refPointNew) = planPath();
  Path path;
  tie(path, refPointNew) = computePath(refPointNew);
  return make_tuple(path, newLane, refPointNew);
}
```
