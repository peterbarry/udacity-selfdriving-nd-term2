easilyeasily# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program


[video_full_run]: ./output-examples/PID-Control-Full-lap-20mph.mov "Video"

[video_no_diff]: ./output-examples/PID-1P-zeroI-zeroD-osc-and-crash.mov "Video"
[video_run_too_fast]: ./output-examples/PID-Fast-run.mov "Video"

For this project, we implemented a PID controller. We tuned the Proportional, Integral and Differential values.

The Proportion value was first varied and resulted in a selected value of 0.2. Higher values caused the car to run off the track.

The following is the video running at 20Mph
![Full Video Run][video_full_run]

To verify the damping capability of the Differential value, a run with zero Differential was carried out, it overshot and oscillated significantly. The following video shows the oscillations

![Video with No differential value, ie - 0.0][video_no_diff]

## Integral Error:
 Through experimental sweep of values, it did not appear as though there was any significant drift error in the simulator. I used a value of zero in the controller.


## Speed :
 The speed of the vehicle  has a significant  impact on the performance of the PID controller. The faster the car the larger the updates in CTE received between samples. The pID model behaved very poorly as was not adaptive to these scenarios. The following video shows the car performance the vehicle throttle set to 0.5 vs 0.2 in the working example. The PID values had worked well at Throttle value of 0.2

![Video with PID running at 50Mph][video_run_too_fast]

## Searching for PID values.

  The simulator used does not easily support automation of searching for PID based on execution of the model. A suggestion would be to allow for the control of the simulator via the messaging interface. This would have facilitated the use of Twiddle or another scheme to search for the best values over a large number of runs. As a result, I used trial and error human searching which undoubtedly  has resulted in poor value selection.


# Origional Udacity Readme.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
