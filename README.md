# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## What is a PID Controller

A PID controller is a feedback control loop controller which calculates continuous error from the desired output, and tries to minimize this error using "Proportional, Integral and Differential" corrections.

* The proportional (Kp) term acts on the current error, i.e. the larger the error, the more the control output.
* Integral term (Ki) acts on the past values of error and helps offset bias.
* Differential term (Kd) acts on the current rate of change of the error. It helps mitigate overshoot and oscillations. On experimenting with hand-tuning, I found out that if this term is too small, the car oscillates on the track and eventually crashes.

## Tuning hyperparamters

I used twiddle algorithm to converge to the paramters in main.cpp. To tune, just set tune_param in Init() to "true". In twiddle mode, the controller runs 800 iterations and resets the simulator each time, to tune the parameters. Once it reaches a tolerance level, the simulator automatically stops tuning further. 


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



