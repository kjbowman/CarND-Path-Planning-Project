# CarND-Path-Planning-Project

Self-Driving Car Engineer Nanodegree Program

## Building and Running the Code

See the original README from Udacity, copied below, for instructions on building and running the code. Note, however, that `CMakeLists.txt` has been modified from the original in order to buld the project from several separately compiled source files, rather than the single `main.cpp` source file provided with the project.

## Overview

## [Rubric](https://review.udacity.com/#!/rubrics/1971/view) Points

### Compilation

#### The code compiles correctly

### Valid Trajectories

#### The car is able to drive at least 4.32 miles without incident

In numerous tests, the car consistently drove more than 4.32 miles without incident. The best run achieved over 50 miles without incident before the simulation was manually terminated, as evidenced by the screen capture below. In the simulation shown, the average speed over the 1 hour and 6 minutes was ust over 45 mph.
![Simulation at 50 miles](images/simulation_50miles.png)

#### The car obeys the speed limit

The car never exceeds the 50 mph speed limit. The configured maximum speed for the car is configured to be 49.5 mph.

#### Max acceleration and jerk are not exceeded

The magnitude of total acceleration rarely exceeds 7 m/s^2, staying below the maximum allowable 10 m/s^2. Likewise, the maximum jerk (up to 10 m/s^3 is allowed) is never exceeded. The magnitude of jerk mostly stays below 3 m/s^3.

#### The car does not have any collisions

The car avoids collisions by maintaining safe distance margins while following a slower vehicle and when changing lanes. In addition, a high cost is assigned to paths that might result in a collision.

#### The car stays in its lane except while changing lanes

The car is never outside a lane (or between lanes) for more than 3 seconds. The car always drives in one of the three lanes on the right side of the double line (i.e. it does not cross into oncoming traffic, nor does it leave the road by driving on the shoulder).

#### The car is able to change lanes

The car is able to - and does - change lanes. The image below shows the car beginning a lane change from the left lane to the center lane.
![Changing lanes](images/lane_change.png)

### Reflection

#### Implementation

##### Code Structure

The source code is divided into four "modules":

- `main.cpp`  the boilerplate code for the main simulation loop
- `vehicle.cpp` implementation of a `Vehicle` class for encapsulating vehicle state (`vehicle.h` contains the definition)
- `cost.cpp`  functions for computing the "cost" of a given path (`cost.h` contains the function prototypes)
- `helpers.cpp` refactored from the supplied `helpers.h` (which remains to define the interfaces/function prototypes) to separate interface and implementation

In addition, the file `parameters.h` contains symbolic constants to aid in readability and to provide a conventient common location to adjust tuning parameters affecting the performance of the path planning algorithm.

##### Overview of the simulation loop

`main.cpp` contains the main simulation loop (lines 64 - 262). The real work begins at line 84 (everything before line 84 is boilerplace code for communicating with the simulator). The main loop executes the following steps:

1. Retrieve the ego vehicle state from the simulator (lines 84 - 92).
2. Retrieve the sensed environment (list of other vehicles as "tracked objects") from the simulator (lines 95 - 104).
3. Compute the projected state of the ego vehicle and the tracked objects (lines 115 - 123).
4. Check for vehicles obstructing the lane(s) ahead of the ego vehicle, if necessary, adjust the ego vehicle's speed to maintain a minimum time gap to the vehicle directly in front of it (lines 125 - 150, 157 - 164).
5. Choose the vehicle state for the next simulation cycle (line 153).
6. Construct or extend the planned path (lines 169 - 243). This portion of the code simply performs the mechanics of computing points ((x,y) coordinates) along a spline, using the suggested spline library.

##### Path Planning and Generation

Step 5, above, is modeled on the _Implement Behavior Planner in C++_ quiz from the classroom.The code for selecting the next best state resides in `vehicle.cpp`. The method `choose_next_state` (lines 52 - 85) gets a list of possible next states and computes a "cost" for each one. The state with the lowest cost is chosen as the next state.

There are three cost functions in `cost.cpp`:

- `inefficiency_cost` (lines 13 - 22), adapted from the _Implement a Second Cost Function in C++_ lesson, penalizes travelling in a slower lane.
- `collision_cost` (lines 26 -61) assigns a penalty for a collision or "near collision" when attempting to change lanes. This cost decreases as the gap between a leading and a following car in the intended lane increases.
- `congestion_cost` (lines 64 - 84) assigns a cost based on the distance to a potential leading vehicle in the intended lane. This cost increases as the "buffer" distance decreases and is intended to favor paths that will allow the ego vehicle to progress further down the road.

The three costs are computed and weighted in `choose_next_state` (`vehicle.cpp` line 74). The cost of a collision is given the highest weight while the inefficiency cost is weighted the least. While it seems obvious that the collision cost should be the most important, the relative weights for inefficiency and crowding were determined experimentally by observing the planning behavior in numerous simulations.

The generation of the chosen path is inspired almost entirely by the _Project Q&A_, using the spline library to generate a smooth path that either follows the center line of the current lane (for lane keeping) or transitions from the current lane into the next lane. The performance of the generated path (i.e. limiting acceleration and jerk) was tuned through extensive experimentation. This is where having all the important parameters defined in one place (`parameters.h`) became extremely beneficial. The following excerpt shows the final tuned values for the various performance-related parameters:
```
#ifndef PARAMETERS_H
#define PARAMETERS_H
...
constexpr int PATH_BUFFER_SIZE = 30;
constexpr int NUM_SPLINE_PTS = 3;
constexpr double SPLINE_PTS_SPACING = 30.0;     // [m]
constexpr double CAR_LENGTH = 4.5;              // [m] (based on avg mid-size car)
constexpr double FOLLOWING_GAP = CAR_LENGTH * 2.5;
constexpr double FOLLOWING_TIME_GAP = 1.0;      // [s]
constexpr double CUTIN_GAP = CAR_LENGTH * 2;
constexpr double SPEED_LIMIT = ROAD_SPEED_LIMIT - 0.5;  // [mph]
constexpr double SPEED_INCREMENT = 0.25;                // [mph/update_rate]
constexpr double LOOK_AHEAD = 500;      // [m]
constexpr double LOOK_BEHIND = 500;     // [m]
...
```

#### Conclusion

---

<p style="text-align: center;">
<b>UDacity's Original README is below</b>
</p>

---

# CarND-Path-Planning-Project

Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

