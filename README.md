# Udacity Self Driving Car Nanodegree
## Path Planning Project 

### Overview

This is a project for Udacity's Self Driving Car Nanodegree. The objective is to build a path planner that creates smooth, safe trajectories for the car to follow. The highway track has other vehicles, all going different speeds, but approximately obeying the 50 MPH speed limit.

The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

---
### Background

**Point Paths**
The path planner should output a list of x and y global map coordinates. Each pair of x and y coordinates is a point, and all of the points together form a trajectory. 

Every 20 ms the car moves to the next point on the list. The car's new rotation becomes the line between the previous waypoint and the car's new location.

The car moves from point to point perfectly in the simulator, there is no need to build a controller for this project.

**Velocity**
The velocity of the car depends on the spacing of the points. Because the car moves to a new waypoint every 20ms, the larger the spacing between points, the faster the car will travel. The speed goal is to have the car traveling at (but not above) the 50 MPH speed limit as often as possible. But there will be times when traffic gets in the way.

**Highway Map**
Inside `/data/highway_map.csv`  there is a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway.

<img src="/images/highwaymap.PNG" width="600">

The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway.

The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change.

**Waypoint Data**
Each waypoint has an (x,y) global map position, and a Frenet s value and Frenet d unit normal vector (split up into the x component, and the y component).

The s value is the distance along the direction of the road. The first waypoint has an s value of 0 because it is the starting point.

<img src="/images/waypoints.PNG" width="400">

The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. The d vector can be used to calculate lane positions. For example, if you want to be in the left lane at some waypoint just add the waypoint's (x,y) coordinates with the d vector multiplied by 2. Since the lane is 4 m wide, the middle of the left lane (the lane closest to the double-yellow dividing line) is 2 m from the waypoint.

If you would like to be in the middle lane, add the waypoint's coordinates to the d vector multiplied by 6 = (2+4), since the center of the middle lane is 4 m from the center of the left lane, which is itself 2 m from the double-yellow dividing line and the waypoints.

**Sensor Fusion**
It's important that the car doesn't crash into any of the other vehicles on the road, all of which are moving at different speeds around the speed limit and can change lanes.

The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car.

Below is a log of the sensor fusion data for two different time steps.

<img src="/images/sensor_fusion.PNG" width="600">

---

### Implementation Summary

At each iteration, the path planner generates `50` trajectory points for the controller to execute the movements. However, in some cases not all trajectory points are consumed by the controller. For example, the controller might have executed `40` points at one iteration, and leftover `10` points.

In this instance, the leftover trajectory points will be passed to the next iteration, at which the path planner will use the 10 points, and plan a new set of 40 trajectory points starting from the ending point of the leftover trajectory.

**Prediction**
In the prediction phase, tracked cars are assumed to keep moving along the same lane, i.e. they are not switching lane. Thus, its future predicted Frenet s value will be its current s value plus its (transformed) total velocity (m/s) multiplied by the time elapsed into the future (s). 

Based on how many leftover trajectory point are available from the last iteration `N`, we project the Frenet s value by adding it with the product of the velocity magnitude and the time elapsed for the leftover points `N * 0.02 s`. Thus, we can get the predicted Frenet coordinate (s and d) of the tracked car at the same time step where we are generating the trajectory for the remaining `50 - N` points.

We loop through the list of tracked cars in the `sensor_fusion` data. The predicted Frenet coordinates for all tracked cars are then used in the Behavior Planning part, which determines what action should our car take.

**Behavior Planning**
For each of tracked car, based on the predicted Frenet d coordinate, we first identify whether the tracked car is

1. on the same lane, or
1. to the left, or
1. to the right
 
with respect to our vehicle.

Then with the predicted Frenet s coordinate and speed of the tracked cars, we implement the following finite state machine:

`IF` one of the tracked cars is on our left `AND`:
1. it's ahead of us with distance < 30m `AND` its speed is <20 ms-1, `THEN` it is not safe to turn left
1. it's behind us with distance < 30m `AND` its speed is >5 ms-1, `THEN` it is not safe to turn left

`IF` one of the tracked cars is on our right `AND`:
1. it's ahead of us with distance < 30m `AND` its speed is <20 ms-1, `THEN` it is not safe to turn right
1. it's behind us with distance < 30m `AND` its speed is >5 ms-1, `THEN` it is not safe to turn right

`IF` one of the tracked cars is on the same lane with us `AND` it's ahead of us with distance < 30m , `THEN`:
1. `IF` it's safe to turn left `AND` we are not on left lane, `THEN` turn left
1. `ELSEIF` it's safe to turn right `AND` we are not on right lane, `THEN` turn right
1. `ELSE` deccelerate

**Trajectory Generation**

We fit a spline using 5 anchor points, which are defined as follows:

1. Second last point from previous planned path
1. Last point from previous planned path
1. 30m ahead of (b) along the center of the same lane using Frenet coordinate
1. 60m ahead of (b) along the center of the same lane using Frenet coordinate
1. 90m ahead of (b) along the center of the same lane using Frenet coordinate

The anchor points are first converted in local coordinate frame of our car before fitting the spline. After the spline is fitted with the transformed anchor points, trajectory points are placed along the fitted spline to ensure it does not exceed the speed limit of 50 mph. 

Lastly, the new trajectory points are transformed back to global coordinate frame and appended to the previous planned path.
