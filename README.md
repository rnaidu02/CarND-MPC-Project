# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## --- Reflections ---

### The model
MPC is designed using Global Kinematic Model that predicts the next state (at state t+1) from state vector at t and the actuator values.
#### Here is the model with the update equations for the next state given the current state

![Model picture](/imgs/model_img.png)

Here is the code snippet inside MPC.cpp
```
// TODO: Setup the rest of the model constraints
// Ref https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/ee21948d-7fad-4821-b61c-0d69bbfcc425
x1 = (x0 + v0 * CppAD::cos(psi0) * dt);
y1 = (y0 + v0 * CppAD::sin(psi0) * dt);
psi1  (psi0 + v0 * delta0 / Lf * dt);
v1 = (v0 + a0 * dt);
cte1 = ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
epsi1 = ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```

#### The state that is passed to the model consists of the following params

`x, y, psi, velocity, cte, and epsi`
![State picture](/imgs/state_img.png)
```
double x = state[0];
double y = state[1];
double psi = state[2];
double v = state[3];
double cte = state[4];
double epsi = state[5];
```

#### Here are the actuators with constraints
![acuators picture](/imgs/actuators_img.png)

One other important component of MPC model is the cost that the solver depends on to optimize the actuator values. Here is the cost formula that is used.
![Cost picture](/imgs/cost_img.png)

The costs considered in the MPC model are the following:
* State cost (CTE, ePsi, velocity)
* Actuator Costs (delta, acceleration)
* Change in actuator costs (difference between prev and current values of delta and acceleration)

I've experimented with diffferent values for the costs multipliers (which one should get more weight towards calculating the cost). What I've observed is that more weight is given to steering value, difference in steering value and CTE. It make sense also to have given weight to the above three as they are key to have the car on the middle of the track and turn the steering angle appropriately.

The multipliers for the cost elements
```
 nCostMultiplierCTE = 200;
 nCostMultiplierEPSI = 15;
 nCostMultiplierV = 10;
 nCostMultiplierDELTA = 2000;
 nCostMultiplierACC = 1;
 nCostMultiplierDELTA_diff = 10;
 nCostMultiplierACC_diff = 1;
 ```

### Timestep Length and Elapsed Duration (N & dt)
I chose timestep Length (N) of 9 and elapsed duration (dt) of 0.1 sec (which amount to total time of close to a second).

The reason I chose dt to be be 0.1 sec because the time lapse between the predicted actuators and the actuator data used inside the emulator is about 100 msec. This way the predicted values can be used in the next update on the emulator screen without delay.

I chose 9 because it is not too high where the number of calculations will be too many for the solver to compute and and the latency will be high.

The other values for N are 15, 20 and 30. With these the MPC way points are not necessarily match with the global way points supplied by the emulator. During the run, the green dots (MPC predicted way points) are only matching with yellow line (emulator supplied way points) around 7 steps. For this reason I reduced them little by little and at N = 9, the MPC worked better up to the speeds of 60 mph.

The other values tried for dt are 0.05 and 0.2. With these 0.2 sec worked ok, but these are not doing any better than dt = 0.1 sec.



### Polynomial Fitting and MPC preprocessing

The following preprocessing was done for the way points (ptsx, ptsy) coming from the emulator. These are in global coordinate system and need to be converted to car's coordinate system so that all of the measurements in same coordinate system for the MPC solver to solve.
```
double x_dot = x*cos(car_psi)+y*sin(car_psi);
double y_dot = -x*sin(car_psi)+y*cos(car_psi);
```
Once the way points are converted to car's coordinate system, the are passed to 3rd degree polynomial fit function inside `main.cpp` to get the coefficients.

```
Eigen::VectorXd polyfit_coeff = polyfit(xcarVals, ycarVals, 3);
```

These coefficients are then used to determine the cte, epsi, and also used inside cost function `FG_eval` to find the cost for cte and epsi that in turn used inside the `ipopt solver` to predict the steering angle and throttle.

```
vector<double> ret_actuators = mpc.Solve(state_vector, polyfit_coeff);
```


### Model Predictive Control with Latency
MPC controller receives the path coordinates from the emulator. MPC controller then converts them to car's coordinate system to predict the actuator values and also the MPC waypoints. MPC controller wait `100 msec` before sending the actuators to the emulator. However MPC controller has a `time step (dt) of 100 msec` and predicts the next way points and corresponding actuators. Since both the delay and the time difference between time steps within vars is 100 msec, the system is synchronized (between the predicted values and when they are received on the emulator side). I've played with receiving actuator values at different time steps and found that the current code inside `MPC::Solve()` worked correctly.

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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
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
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
