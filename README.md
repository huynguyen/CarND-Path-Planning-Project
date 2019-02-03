## README

## Model Path Generation
We start with `CarsInLane` that takes the sensor fusion data and partitions the other cars into the `left = 0`, `middle = 1`, `right = 2` lanes.


We then try to decide what the best action can be based on our state. This is decided via the `BestAction` function. This function first tries to compute a score for each lane using a heuristic of which lane is the clearest. This is determined by the distance to the closet car ahead of our car in that lane. There is also a small boost to keep the current lane in the case of ties.

Once we determine what the `best_lane` is we need to see if it's safe to switch. We need a minimum speed and to determine if there is sufficient space in the lanes we are trying to move to. If we meet all these conditions we return one of the change lane actions.

If we can't change lanes we need to determine if there if we are too close and need to slow down. If we are slow down or if there is now space we need to speed up.

Once we determine the best action, we try to perform it. For changing lanes we modify the lane variable which will later on be used to generate waypoints on the spline. Speeding up or slowing down changes the ref_vel.

We try to use previous points for our spline to make it smooth. If there are none we generate a previous one based on our trajectory. We then add a few waypoints 30, 60, 90m out factoring in our lane which we might have just modified and the map waypoints.

We then shift the spline waypoints to the car reference which helps spline avoid cases where x could have multiple values y values.

Reuse the path points we generated previously but haven't yet consumed so the transition to our new path is smooth. We generate some new points based on the new spline waypoints and then space them sufficiently far apart to achieve the speed we want and not acceed Max Acceleration and Jerk.

## Possible Improvements
### React faster
Sometimes a car might swerve in front of us, in these situations the way we change speed should be more dynamic. We could possibly improve this by actually modifying the previous points instead of always adding on top of them.
### Maintain speed
Sometimes the car slows too much and then speeds back up to max instead of simply trying to maintain some speed that will keep us a safe distance behind the car in front of us. We could help some of this with the react faster mentioned above and a buffer distance where we aren't increasing or decreasing our speed.
### Indecisiveness
There should be some state tracking where we punish movements that are cycling such as when the best lane is ambiguous due to two cars trading positions slightly in other lanes. 
### More complex Actions.
Situations where there is a better lane but we can't switch to it because there is a car blocking. For example we are in lane 0 and we want to switch to 2, but there is no gap in lane 1 could be handled better by trying to slow down and/or find a gap where the lanes are clear.