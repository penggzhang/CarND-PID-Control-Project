# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Reflection

####1. Describe the effect each of the P, I, D components had in your implementation.

Proportional control gives a correction steering based on the cross track error, and thus leads the vehicle to turn to and approach the target track, which is the middle line of the road. However, P control will cause overshoot, waving around or not going converge. To counter such overshoot, differential control based on the temporal difference of cross track error comes into play. D control makes the vehicle gracefully approach target track and converge. Finally, integral control deals with vehicle drift by taking the integral of cross track error to a correction steering.

[My project video](https://youtu.be/Zi3O9A3XHxg)

Again, as seen from the video, P control always brings the vehicle back to the target middle line of the road, D control smooths the turning and make it converge, I control corrects any drift.

####2. Describe how the final hyperparameters were chosen.

#####1) Guess initial values
First of all, the P control parameter, Kp, is roughly estimated. Given the distance from the edge to center of the road is about 2.2(m),  and the maximum steering angle is 1 or -1 in radian, a Kp around 0.4 (= 1 / 2.2) was guessed. Then manually tried some values for other two parameters through simulations, and correspondly Kd of 5 as well Ki of 0.0001 were guessed too.

#####2) Twiddle to fine tune the parameters
Based on an initial guess for Kp, Kd, Ki as 0.35, 5.0, 0.0001 respectively, twiddle method was conducted on the first 1,500 step simulation. Root mean square (cross track) error was chosen as the criterion to compare the goodness of hyperparameters. After nearly 200 iterations, a best RMSE was found at the parameters of Kp = 0.993922, Kd = 11.0414, Ki = 0.00043469. And the best RMSE there was 0.0915366(m).

#####3) Run simulation to verify this optimization
Run the simulation with the fine-tuned parameters. It works and verifies that the optimization work was effective.