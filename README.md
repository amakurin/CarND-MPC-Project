# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
---
The goals / steps of this project are the following:

* Apply MPC to steering and throttle of a simulated car.
* Compensate latency in control system (100ms).
* Tune hyperparameters of controller so that car could autonomously drive a lap without living the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). 

Additional goal was to maximize speed of the car keeping driving stable (with minimum oscillations). 

[//]: # (Image References)

[result]: ./result/final.gif "Result"
[classic_model]: ./result/classic_model.png "Classic model"
[simplified_model]: ./result/simplified_model.png "Simplified model"
[model_init]: ./result/model_init.png "Model"
[psidot]: ./result/psidot.png "Psi dot equations"
[model_cont]: ./result/model_cont.png "Model continuous"
[model_descrete]: ./result/model_descrete.png "Model descrete"

## Demo

![result][result]

Here's the [link to full video](https://youtu.be/fSjnHh_LDdM).

## Model

MPC stands for Model Predictive Control, so let's start with the model.

In this project [kinematic bicycle model](http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf) is used. Here is classic drawing for derivation of model's equations:

![classic_model][classic_model] 

**x**, **y**, **psi** and **v** are state variables of the vehicle. Control inputs are **delta** (steering angle) and **a** - acceleration that isn't shown on drawing, but we assume it is directed along the vehicle axis.

Drawing shows that velocity vector **v** forms angle **beta** with vehicle main axis. This is important for long vehicles or for vehicles with ability to steer by front and rear wheels, or drift. 

None of this is applicable to project, so we can assume  velocity vector is directed along main axis of vehicle. So **beta** is zero and drawing for this case looks like this:

![simplified_model][simplified_model]

Using trig and formula for [angular velocity](https://en.wikipedia.org/wiki/angular_velocity), we can formulate following system of equations:

![model_init][model_init]

where **a** is vehicle acceleration.  

From right triangle with catheti of **R** and **Lf** we can derive equation for **R** , substitute to psi dot equation, and assume tan(delta) = delta for small values:

![psidot][psidot]

Note about small delta assumption: maximum posible steering angle for project is 25 degrees, which is 0.436332 radians; for that value the error is tan(0.436332)-0.436332 = 0.029975277097

So the final system of equations for project model is: 

![model_cont][model_cont]

Or in discrete case:

![model_cont][model_descrete]


## Prediction

Discrete model equations are used for latency compensation and solving for optimal trajectory.

### Latency in control system

Appling control takes time and produces effect of latency. Because of latency, the state used for optimizing trajectory and control could differ to much from actual state at the moment of actuation. It is especially important during tough actuation. You can see this on [0:21](https://youtu.be/fSjnHh_LDdM?t=21) of project video: yellow line is a reference trajectory, it is build by transforming reference waypoints from world to vehicle coordinate space, using predicted state. But even with prediction on a tough turn we see drift of this line. This drift is caused by latency (or\and inacurate prediction) - the state used for coordinate transformation differs too much from the actual state on the moment of line rendering.

The project simulates worst case of latency - just thread.sleep(100), which means that controller can't neither actuate nor read state during 100ms.

To fight the latency, firstly, latency is estimated, then actual state on the moment of actuation is predicted using model equations, and after that, all calculations are held using the predicted state.

### Optimization of trajectory and control

The task of vehicle control is to drive vehicle as close as possible to reference trajectory. 

Simulator used with project doesnot provides entire trajectory, but few reference waypoints in world coordinates. So th first task of controller after state prediction is to calculate reference trajectory. This is done by first transforming waypoints to vehicle reference frame and then by fitting 3d order polynomial with resulting points.

MPC optimization task formulated as follows: using model equations, find the control parameters (**delta** and **a**) as well as resulting trajectory (state parameters **x**, **y**, **psi**, **v** at each actuation step) such that minizes value of cost function.

Project cost function sumarize folllowing:

* cross-track error (CTE) cost;
* heading error (EPsi) cost;
* velocity error cost;
* high steering angle cost;
* sharp steering cost;
* sharp acceleration\deceleration cost.

These costs are computed as product of corresponding errors and proportional coefficients. These coefficients are hyperparameters of MPC controller.

Another important hyperparameters are 

* **T** - prediction horizon in seconds; 
* **N** - number of timesteps in the horizon;
* **dt** - prediction timestep between actuations.

Actually, N = T/dt. 

#### Prediction horizon

In project case, prediction horizon should be selected such that predicted trajectory doesn't go much beyond the furthest waypoint. We don't have actual trajectory, and fitted polynomial gives inacurate estimation beyond the fitting.  

#### Prediction timestep

Prediction timestep should be small enough to minimize discretization error.

But, except discretization error, there is no sence in using value for dt much lesser then overall latency. If so - solver will predict actuations which will never applied because of latency, so it is just wasting of computation time and increasing of latency.

For project dt=0.2 and N=7 were chosen. This gives 1.4 seconds of prediction horizon, which a little beyond the furthest waypoint on high speeds.

#### Speed setpoint

For speed setpoint controller algorithm analyzes provided waypoints to find maximum heading deviation, then manhattan distance is computed to point with maximum heading deviation. The ratio of this distance to heading deviation is used to compute maximum speed for current state. Then this value is used to compute speed setpoint with respect to current EPSI.

Project video was created by running controller with following parameters:

```
./mpc V_MIN=25 V_SAFE=35 V_MAX=70
```
on laptop 2.5GHZ Core i7, 16GB DDR3.

Default parameters are not so agressive to let controller drive safely on modest machines.
  

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
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
