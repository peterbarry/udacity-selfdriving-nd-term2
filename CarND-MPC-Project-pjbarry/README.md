# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## MPC Contoller
Udacity Project 5 Term 2. The project involves developing a Model Predictive Controller. The controller receives way point information from the Udacity car simulator and proves steering and accelerator input.

## Program Steps

1.  Convert the waypoint inout from the simulator to car ego centric.
2. Curve fit a line from the vehicle to waypoints.
3. Calculate the state vecotr for optimizer based on predicted values at a later time. Predicted values
    - X
    - Y
    - PSI,
    - Velocity
    - Cross Track Error
    - Epsi - vehicle orientation.
4. Solve to produce steering angle and accelerator using costs (see below)    
5. Take output of solver and send to car simulator.

## Costs

The following costs were used to tune the are key to the performance of the system:

The first cost : is the cross track error, striving to keep the vehicle close to the waypoints.
```
  // cross-track error.
  fg[0] += 1000 * CppAD::pow(vars[cte_start + t], 2);
```

Cost to ensure vehile points in line with the waypoints.
```
  // Heading Error
  fg[0] += 2000 * CppAD::pow(vars[epsi_start + t], 2);
```

The velocity error strives to increase the speed of the car.
```
  // Velocity Error vs max target speed.

  fg[0] += 1 * CppAD::pow(vars[v_start + t] - ref_v, 2);
```
If the vehilces has a large cross track error slow down.
```
  // slow down if large error.
  fg[0] += 0.5 * CppAD::pow(vars[cte_start + t] * vars[v_start+t], 2);
```

The following reduce the likelihood of very aggressive changes in steering or accelerator outputs over time.
```
  fg[0] += 1 * CppAD::pow(vars[delta_start + t], 2);
  //Smooth the actuation of the accelerator
  fg[0] += 100 * CppAD::pow(vars[a_start + t], 2);

  // Add an extra cost for aggressive steering at high speed - unused
  fg[0] += 0 *CppAD::pow(vars[delta_start + t] * vars[v_start+t], 2);
```

// Ensure that there are not large steps in actuation.
```
  fg[0] += 10  * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
```


## Timeline/Steps
 A Delta T of 100ms and 10steps was used for the MPC. That value was tuned and found to be a good value. Initially a large number of steps was used but the execution time was very long 200ms. Given that I had not included this time in the future values predictions the system was very erratic (see lessons  below

## Lessons
The MPC was very difficult to tune at first, I found that the execution time of the MPC optimization algorithm ipopt took a significant  amount of time to provide a solution. In a 2015 macbook air, the algorithm took between 40 and 80ms. The converge time was longer at the aggressive turns portions of the track. This resulted in the predicted values for the car calculated to have significant error by the time the values were used post optimization for actuation. I added code that approximated the cost of the algorithm and add the time to the predicted variables of the vehicle, including actuation delays.

## Curve Cutting.
 In an effort to drive a more natural looking line, I modified the future predicted Y value depending on a very crude estimate  of the extent  of the forward road curvature.  The behavior is to "hug" the side of the road for an upcoming corner. The MPC costs and parameters were tuned with this enabled. The indication is simply the change in the y value 100points ahead. The y change is limited to +- 1.0 to stay on the track.

 ```
 double curve_indicator = polyeval(coeffs,100); // whats the value of y ahead./should be speed related

 future_y -= curve_indicator/10; // shift to side of curve
 if (future_y >= 1.0 )
   future_y = 1.0;
 if (future_y <= -1.0)
   future_y = -1.0;

 ```


## Data received from simulator:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.



---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
