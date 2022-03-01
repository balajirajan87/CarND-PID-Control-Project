# PID Controller project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview

PID stands for Proportional-Integral-Derivative. These three controllers are combined in such a way that it produces a control signal. This is how the vehicle uses steering, throttle, and brake to move through the world, executing a trajectory created by the path planning block.

In this project we will revisit the lake race track from the Behavioral Cloning Project. This time, however, we will implement a PID controller in C++ to maneuver the vehicle around the track! The simulator will provide the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

I am going to discuss based on how to use PID for steering angles, and throttle

## PID Controller for steering  - components

### Cross Track Error
A cross track error is a distance of the vehicle from trajectory. In theory it’s best suited to control the car by steering in proportion to Cross Track Error(CTE).

### P component
It sets the steering angle in proportion to CTE with a proportional factor tau.

```
p_error = tau * cte
```

In other words, the P, or "proportional", component has the most directly observable effect on the car’s behavior. It causes the car to steer proportional (and opposite) to the car’s distance from the lane center(CTE) - if the car is far to the right it steers hard to the left, if it’s slightly to the left it steers slightly to the right. Having Proportional controller alone to steer the vehicle could cause the vehicle to oscillate to and fro on the lateral direction. Because even though after applying the corrective steering, and when the vehicle is closer to the track reference, ideally one would expect the vehicle to be oriented along the refernce line but that is not the case. Vehicle would actually be mis oriented from the reference line and this causes the vehicle to overshoot.

### D component
It’s the differential component of the controller which helps to take temporal derivative of error. This means based on the gradient of error change, this helps to reduce the amount of steering, and thereby helps to allign / orient the car along the reference Trajectory.

In other words, the D, or "differential", component counteracts the P component’s tendency to ring and overshoot the center line. A properly tuned D parameter will cause the car to approach the center line smoothly without overshooting.

```
diff_cte = cte - prev_cte
prev_cte = cte
d_error = tau_d * diff_cte
```

### I component
It’s the integral or sum of error to deal with systematic biases.

In other words, the I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift , but I believe that in this particular implementation the I component particularly serves to reduce the CTE around curves.

```
int_cte += cte
i_error = tau_i * int_cte
```

And combination of these we can get PID controller to control the steering value. Negative signs infront of each terms is used to steer the vehicle in a direction opposite to the error. If for example the Error is positive (meaning the vehicle is located on the positive y axis), then -ve steer value causes the vehicle to be steered to the right, and vice versa.

```
steer = -p_error - d_error - i_error

```

Parameter optimisation can be done manually or using Twiddle algorithm. Twiddle Algorithm is basically a gradient descend algorithm where you either increase or decrease your parameter based on the intensity of your Average Cross Track error. when error is lesser than the best error, you step up the paranmeters. when the error is greater than the best error, you step down the threshold.

Pseudocode for implementing the Twiddle algorithm is as follows:

```
function(tol=0.2) {
    p = [0, 0, 0]
    dp = [1, 1, 1]
    best_error = move_robot()
    loop untill sum(dp) > tol
        loop until the length of p using i
            p[i] += dp[i]
            error = move_robot()

            if err < best_err
                best_err = err
                dp[i] *= 1.1
            else
                p[i] -= 2 * dp[i]
                error = move_robot()

                if err < best_err
                    best_err = err
                    dp[i] *= 1.1
                else
                    p[i] += dp[i]
                    dp[i] *= 0.9
    return p
}
```

## PID Controller for Speed control -  components

### Velocity Error
velocity error is the difference between the Target vehicle speed and the Current speed. The Target Speed is defined as a single modifiable value. Currently i have set the Target Speed as 30 MPH. 

### Longitudinal Trajectory planner:
Providing a constant throttle value, could result in heavy accelerations when starting the vehicle from standstill which inturn makes controlling the vehicle laterally a difficult job. instead i thought of increasing the speed with a constant acceleration based on difference between the Current and the target speed. The Acceleration component is derived as the factor multiplied with Difference between current and the Target velocity, as mentioned below. This would be added to the existing velocity to get the new velocity.
```
double PID::calc_traj(double tar_speed){
    if (!is_traj_initialized){
        mod_speed = 0.0;
        mod_accel = 0.0;
        is_traj_initialized = true;
    }
    double kp_speed = 0.05;
    double kp_accel = 0.05;
    double tar_spd_ = tar_speed / 2.24;
    
    double tar_accel_raw = fmax(fmin(kp_speed * (tar_spd_ - mod_speed), 0.1), -0.1);
    double tar_jerk_raw = fmax(fmin(kp_accel * (tar_accel_raw - mod_accel), 0.05), -0.05);
    mod_accel = mod_accel + tar_jerk_raw;
    mod_speed = mod_speed + mod_accel;
    return mod_speed * 2.24;
```
So i allow a max / min acceleration of 0.1 / -0.1 and max / min jerk of 0.05 / -0.05.
The output generated from this function is the modified target speed that increases smoothly and settles smoothly near the original target speed. This Speed calculated acts like a reference speed to follow for a smooth longitudinal control.


## Finding the right coefficients - Manual and Automatic Tuning of PID Hyper-parameters.

The intial value for Kp, Ki, Kd selected using trail and error method. It is a simple method of PID controller tuning. In this method, first we have to set Ki and Kd values to zero and increase proportional term (Kp) until system reaches to oscillating behavior. Then Kd was tuned to reduced oscillation and then Ki to reduce steady-state error

Initially after manual tuning i set the coefficients as below

```
{0.01, 0.0001, 1.0}

```
Then I decided to use Twiddle algorithm to optimise these coefficents further. I created a function call to twiddle from the main.cpp. when the "twiddle_avail" variable was set to FALSE, i ran the normal PID controller by calling the "pid.UpdateError( )", and "pid.totalError( )" functions.
When "twiddle_avail" was set to TRUE, i start to calculate the total cross track error, and run the pid controller by calling the "pid.UpdateError( )", and "pid.totalError( )" functions, untill the number of iterations reach the max_iter value.
```
total_cte += pow(cte,2);
UpdateError(cte, err_spd);
control_output = TotalError();
```
each time when the Max Iter value is hit we check the average cross track error, and when the error is lesser than the best error we save that error as the new best error and we increament one of the parameters by a certain factor as explained below:
```
if (first_change == true)
{
    p[p_iterator] += dp[p_iterator];
    std::cout << "step change: p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << endl;
    first_change = false;
}
else
{
    error_twiddle = total_cte / max_iter;
    if (error_twiddle < best_error && second_change == true)
    {
        best_error = error_twiddle;
        best_p[0] = p[0];
        best_p[1] = p[1];
        best_p[2] = p[2];
        dp[p_iterator] *= 1.1;
        fine_tune_count += 1;
```
when the error is greater than the best error we decrement the parameters. again with the new parameters we check how the pid controller performs and when the error is lesser than or greater than the previous best error we are either going to increment / decrement the parameters. 
```
if (second_change == true)
{
    p[p_iterator] -= 2 * dp[p_iterator];
    std::cout << "correction: p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << endl;
    second_change = false;
}
else
{
    if (error_twiddle < best_error)
    {
        best_error = error_twiddle;
        best_p[0] = p[0];
        best_p[1] = p[1];
        best_p[2] = p[2];
        dp[p_iterator] *= 1.1;
        fine_tune_count += 1;
    }
    else
    {
        p[p_iterator] += dp[p_iterator];
        dp[p_iterator] *= 0.9;
        fine_tune_count += 1;
```
so each time the simulator is called the main loop calls the twiddle and twiddle does the optimization tasks. 
i set the max_iter to 1. that means each time the cte would be equal to the average cte and the parameter optimization would be done during the each call. This i found to be much better rather than setting the same to either 100 or 600. 
The initial dp was set as below:
```
0.01, 0.0, 0.1
```
when the sum of dp coefficients got below a certain minimal thresholds, which indirectly means that the controller has started to settle and has already found the best coefficients, i set the twiddle_avail to FALSE and made the controller run with a normal PID control.
Finally we got the optimised coeffients as below:

```
0.194384, 0.0, 1.84692,
```

For Speed / velocity control i used a PI controller for smoothly achieving a target speed of 30 mph.
the coefficients were:
```
0.5, 0.001, 0.0
```

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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

