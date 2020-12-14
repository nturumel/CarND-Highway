# TOC:

- [CarND-Path-Planning-Project (Self-Driving Car Engineer Nanodegree Program)](#carnd-path-planning-project--self-driving-car-engineer-nanodegree-program)
    + [Project Description](#project-description)
- [CarND-Path-Planning-Project](#carnd-path-planning-project)
    + [Simulator.](#simulator)
    + [Goals:](#goals)
    + [Requirements:](#requirements)
      - [Simulator:](#simulator)
      - [Build:](#build)
      - [The map of the highway:](#the-map-of-the-highway)
      - [Build Instructions:](#build-instructions)
    + [Challenges:](#challenges)
    + [Inputs Provided:](#inputs-provided)
      - [Sensing Data:](#sensing-data)
      - [Previous Path Data:](#previous-path-data)
      - [Main car's localization Data (No Noise):](#main-car-s-localization-data--no-noise)
      - [Previous path's end s and d values :](#previous-path-s-end-s-and-d-values)
    + [Solution:](#solution)
      - [Behavior Planner:](#behavior-planner)
      - [Trajectory Planner](#trajectory-planner)
    + [Result](#result)
      - [Improvements](#improvements)

# CarND-Path-Planning-Project (Self-Driving Car Engineer Nanodegree Program)

### Project Description

This Project is set on a virtual environment consisting of a track and multiple vehicles. The track is 6946m long, has 6 lanes, each 4m wide, 3 on each side of the yellow line. There is one vehicle on the track that we control. The rest of the traffic is populated at random locations, have speeds ranging from 40 to 60 MPH and there behavior is controlled by the simulator. 

# CarND-Path-Planning-Project

Self-Driving Car Engineer Nanodegree Program

### Simulator.

You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:

```shell
sudo chmod u+x {simulator_file_name}
```

### Goals:

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

1.  Navigate our car safely across the track.
2.  Strictly adhere to the speed limit of 50 MPH.
3.  Strictly adhere to maximum jerk of 10 m/s^2 and a jerk of 10 m/s^3.
4.  Not spend more than 3 seconds outside the 3 lanes, meant for our side of the traffic.
5.  Change lanes when required.
6.  Sense other cars and avoid collision.

### Requirements:

#### 	Simulator: 

To run the simulator on Mac/Linux, first make the binary file executable with the following command:

```shell
sudo chmod u+x {simulator_file_name}
```

#### 	Build:

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

    ```bash
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

​	

#### 	The map of the highway:

Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to 	get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### 	Build Instructions:

Clone this repo:

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./path_planning`.

### Challenges:

Since the simulator governs the traffic on the track, it is impossible to predict what the other vehicles will do next. Also, the implementation of the software requires that our car be supplied a list of x and y coordinates on the road, car reaches each point in 0.02 seconds. This list of points in the only input that the car takes, so it is a bit challenging to pass on precise information about the desired speed and lane of the car through of a set of points.  

### Inputs Provided:

#### 	Sensing Data:

The simulator provides us with sensing data. The sensing data is a 2D vector of cars containing localization data of the surrounding cars. The data of each car is stored in the following order->  [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x 	velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates]

#### 	Previous Path Data:

The simulator usually asks for the next set of instructions before completely executing the previous set. We get access to the following data:

| Name                | Description                                                  |
| ------------------- | ------------------------------------------------------------ |
| ["previous_path_x"] | The previous list of x points previously given to the simulator |
| ["previous_path_y"] | The previous list of y points previously given to the simulator |

#### 	Main car's localization Data (No Noise):

We get access to the car's localization data:

| Name      | Description                                |
| --------- | ------------------------------------------ |
| ["x"]     | The car's x position in map coordinates    |
| ["y"]     | The car's y position in map coordinates    |
| ["s"]     | The car's s position in frenet coordinates |
| ["d"]     | The car's d position in frenet coordinates |
| ["yaw"]   | The car's yaw angle in the map             |
| ["speed"] | The car's speed in MPH                     |



#### 	Previous path's end s and d values :

We also have access to the location that the car will after the completion of the previous instruction.

| Name           | Description                                     |
| :------------- | ----------------------------------------------- |
| ["end_path_s"] | The previous list's last point's frenet s value |
| ["end_path_d"] | The previous list's last point's frenet d value |

### Solution:

The solution consists of two parts. The behavior planner and the trajectory planner.

#### 	Behavior Planner:

The behavior planner takes in the sensing information and our car's localization data, and suggests the most suitable action, intended lane and intended speed.

Immediately upon receiving the sensing data, the Behavior planner calculates a suitable speed for each lane, that is the max speed the car can safely travel.

The behavior planner calculates the best possible action based on the following costs:

| Name               | Description                                                  |
| :----------------- | ------------------------------------------------------------ |
| Lane Change Cost:  | The car cannot change lane within 10 seconds of a previous lane change. As changing lanes affects comfort, it returns a value based on the difference between the current lane and intended lane. |
| Speed Change Cost: | This returns the cost of the difference between the current speed and the target speed of recommended action. |
| Speed Cost:        | We want to drive fast. This cost returns a lower cost if we are travelling close to the maximum allowable speed, and a higher cost if we are driving really slow. |
| Safety Cost:       | This is perhaps the most important cost, a lane change will not be considered if this cost evaluation does not return 0. It considers if the vehicle in front in the current lane is too close and if the closest front and back vehicle are too close in the intended . |
| Buffer Cost:       | This returns a cost based on how far apart the other vehicles are in the intended lane. The lane with the lowest cost is selected for the next action. |

#### 	Trajectory Planner

The trajectory planner gives us a suitable trajectory from our current state to the state recommended by the behavior planner. It does so in accordance to the max acceleration and jerk conditions set in this project. To avoid exceeding the acceleration when changing lanes in a sharp corner, we decelerate. A spline is generated from the car's current d, s value to a the d value of the desired lane's center and a suitable s value ahead. Then the following linear interpolation technique is used to get the set of x and y coordinates to move the car.

```c++
while(nextVals[0].size() < _newSize)
    {
        acc = getAcc(_refVel, desiredVel);
        
        // not really required except for the curves... it seems, just make max 0.7 ...
        if(_laneChange)
            acc -= (0.02);

        // update the speed
        _refVel = std::max(0.5, _refVel + acc);
        _refVel = std::min(_h->_maxVel, _refVel);

       
        double N = (targetDist / (0.02 * _refVel));
        double x = xAddOn + targetX / N;
        double y = sp(x);
        
        xAddOn = x;
        car2Global(x, y, refYaw, refX, refY);


        nextVals[0].push_back(x);
        nextVals[1].push_back(y);

     }
```



### Result

The car is able to successfully navigate the track, it maintains good speed, and changes lanes suitably. The maximum successful distance recorded is 6.5 miles. 

https://youtu.be/SHhATswKndI

![Highway](Highway.gif)

#### Improvements

1.  The car cannot do two successive lane changes.
2.  The car can get stuck behind a slow moving car and may not change lanes if the cars in the next lane and really close, it does not slow down, wait for the other cars to pass and then change lanes.



# 