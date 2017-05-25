# INTRODUCTION

This is my turn-in code for one of the project in partial fulfillment of the requirements for Udacity's self-driving car Nanodegree program. In this project, I have implemented a Model Predictive Control software pipeline to drive a car around a track in a simulator. There is a 100 millisecond latency between actuation commands on top of the connection latency.

### IMPORTANT: Please view my screen recording of using my code for the vehicle to drive around the track for several laps:
- https://www.youtube.com/watch?v=75ylhM0QsXQ&feature=youtu.be
- The MPC trajectory path is displayed in green, and the polynomial fitted reference path in yellow.
- I used 640 x 480 screen resolution, with a graphic quality of fastest 
- Tested in macOS Sierra Version 10.12.4 Macbook Pro Mid 2014. 2.6 GHz Intel Core i5.

![MPC](https://github.com/mithi/mpc/blob/master/img/pic.png)

**According to wikipedia:**
> Model predictive controllers rely on dynamic models of the process. The main advantage of MPC is the fact that it allows the current timeslot to be optimized, while keeping future timeslots in account. This is achieved by optimizing a finite time-horizon, but only implementing the current timeslot. MPC has the ability to anticipate future events and can take control actions accordingly.

**Here is an overview of the MPC approach as provided by Udacity:**
![MPC](https://github.com/mithi/mpc/blob/master/img/mpc.png)
![MPC](https://github.com/mithi/mpc/blob/master/img/mpc-algo.png)

In my own words, the MPC method can anticipate future events because we have an idea of what is probably gonna happen if we choose to do what we are going to do. This is because we have a model of how things work in our world (like physics for example). We can anticipate future events based on our current plan of action and also anticipate our next plan of action based on the result of the current plan.

- Here is a nice series of videos that give a nice overview about the concept behind MPC as well as an in depth discussion:
-- https://www.youtube.com/watch?v=4kCcXGDvjU8&list=PLs7mcKy_nInFEpygo_VrqDFCsQVnGaoy-
- For more information, you can check the initial project repo from Udacity here:
-- https://github.com/udacity/CarND-MPC-Project
- The simulator used can be downloaded here:
-- https://github.com/udacity/self-driving-car-sim/releases
- The flow of the code is largely based on Udacity's sample code and quizzes
-- https://github.com/udacity/CarND-MPC-Quizzes


# THE STATE VARIABLES

**The state variables for the vehicle are the following:**

### `px`
 - The current location in the x-axis of an arbitrary global map coordinate system
### `py`
 - The current location in the x-axis of an arbitrary global map coordinate system
### `psi`
 - The current orientation / heading of the vehicle
### `v`
 - The current velocity/speed of the vehicle

This is given to us by the simulation every time we ask for it. In addition to this, we are also given a series of `waypoints` which are points with respect to an arbitrary global map coordinate system we can use to fit a polynomial which is a function
that estimates the curve of the road ahead. It is known that a 3rd degree polynomial can be a good estimate of most road curves. The polynomial function is with respect to the vehicle's local coordinate.

# CTE AND EPSI

![ERROR](https://github.com/mithi/mpc/blob/master/img/errors.png)

We can compute for the errors which is the difference between our desired position and heading and our actual position and heading:

### `cte`
 - This is the cross track error which is the difference between our desired position and actual position. We can use our fitted polynomial at point px = 0 to get the position where we should
 currently be. `cte = f(0)`

### `epsi`
 - This is the orientation error which is the difference between our desired heading and actual heading. Our desired orientation is the heading tangent to our road curve. This can be computed using an arctan to the derivative of the fitted polynomial function at point px = 0 to get the angle
 to which we should be heading. ```epsi = arctan(f`(0)) where f` is the derivative of f```

# ACTUATIONS: STEERING, THROTTLE, AND BRAKE
From this we use Model Predictive Control to give what what is our best course of action. There are two modes of *actuation* we can use to control our vehicle.

### `delta`
  - This is the steering value which represents the angle of which we turn our vehicle, which I suppose is the angle of the vehicle's tires. The angle is restricted to be between -25 and 25 degrees but is mapped to the values between -1 and 1. **In my code, I have restricted this to only be between -0.75 and 0.75** as I which to be conservative as opposed to aggressive in our steering.
  Since we can plan ahead, we shouldn't be needing to suddenly steer a large angle.

## `a`
  - This is the *throttle* or *brake* value which represents the acceleration or deceleration of our vehicle. In an actual vehicle, this is controlled by the brake pedal. The simulator expects values between -1 and 1. Negative values represents braking and positive values represents throttle.
  **In my code, I have restricted the range to only be between -0.5 and 1** as ideally we shouldn't be suddenly pressing the brakes all of a sudden since we are able to plan ahead.

# KINEMATIC MODEL
So based on Physics, here is a simplified version of how the world (with our vehicle in it) works. How the state variables change based elapse time `dt` and our actuations `delta` and `a`.
We call it our *kinematic* model.
```
px` = px + v * cos(psi) * dt
py` = py + v * sin(psi) ( dt)
psi` = psi + v / Lf * (-delta) * dt
v` = v + a * dt

Lf - this is the length from front of vehicle to Center-of-Gravity
```

We can also predict the next `cte`, and `epsi` based on our actuations.
```
cte` = cte - v * sin(epsi) * dt
epsi` = epsi +  v / Lf * (-delta) * dt
```

Recall from the discussion:
```
cte = py_desired - py
epsi = psi - psi_desired
py_desired = f(px)
psi_desired = atan(f`(px))
where f is the road curve function
      f` is the derivative of f
```


# Cost Function and Penalty Weights

Given the constraints of our model, we should have a good objective such as a cost function to minimize. Here are the factors we should consider:

- We should minimize the cross track error `cte`, we want to be in our desired position
- We should minimize our heaidng error `epsi`, we want to be oriented to our desired heading
- If possible, we want to go as fast as we can `v = 100 mph`
- We don't want to be erratic on our driving IE:
- 1. We don't want to steer if we don't really need to
- 2. We don't want to accelerate or brake if we don't really need to
- 3. We don't want consecutive steering angles to be too different
- 4. We don't want consecutive accelerations to be too different

So mathematically it should be like:

```
cost = A * cte^2 + B * epsi^2 + C * (v - vmax)^2 +
       D * delta^2 + E * a^2 + F * (a` - a)^2 +  G * (delta` - delta)^2

... integrated over all time steps
```

The penalty weights are again determined through trial-and-error, taking into consideration
that our primary objective is to minimize `cte` and `epsi` so that should hold the highest weight.
I also increased  the weight cost for consecutive steering angles when I notice that
the path that the vehicle was taking is somehow jagged. The least of our concerns is going
as fast as possible. I also had to increase the weight cost a little bit for the consecutive
acceleration difference because we don't want to turn too late at road curves.

Ultimately here's the weights I ended up with:

```
A = 1500
B = 1500
C = 1
D = 10
E = 10
F = 150
G = 15
```

# Time Step Length and Frequency

The timestep length `N` is how many states we "lookahead" in the future and the time step frequency `dt` is how much time we expect environment changes.  I chose a `dt = 0.1 seconds` because the that's the latency between actuation commands so it seemed like a good ballpark.  It doesn't make sense to *look too far to the future* because that future might not be as we expect it, so we must not calculate too much before getting feedback from the environment. Also, it will take a long time to look for the best move as I have set this to a time limit of 0.5 seconds. If the `N` is too small, this makes us too short-sighted which defeat the purpose of planning for the future. If the `N` is too small we might not be able to take advantage of looking ahead to plan for curves that we might not be able to do with simpler and less sophisticated control methods like PID. I started with `N = 6` because that's the number of `waypoints` given to us but looking at the displayed green line at the simulator makes too short to plan for curves. At `N = 15` I noticed that given the time limit of 0.5 seconds, I running out of time to minimize the cost objective. With trial-and-error I found that `N = 10` was good.

# Polynomial Fitting and MPC Preprocessing

The waypoints to estimate the road curves is given at an arbitrary global coordinate system, so I had to transform them to the vehicle's local coordinate system.

```cpp
 for(int i = 0; i < NUMBER_OF_WAYPOINTS; ++i) {
    const double dx = points_xs[i] - px;
    const double dy = points_ys[i] - py;
    waypoints_xs[i] = dx * cos(-psi) - dy * sin(-psi);
    waypoints_ys[i] = dy * cos(-psi) + dx * sin(-psi);
  }
```

I used this to get a 3rd order polynomial as an estimate of the current road curve ahead.
To get the fitted curve, I used a function adapted from here: https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716

We also have to generate the current errors as discussed above.

Here it is in code:

```cpp
  // current CTE is fitted polynomial (road curve) evaluated at px = 0.0
  // f = K[3] * px0 * px0 + px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];
  const double cte = K[0];
  // current heading error epsi is the tangent to the road curve at px = 0.0
  // epsi = arctan(f') where f' is the derivative of the fitted polynomial
  // f' = 3.0 * K[3] * px0 * px0 + 2.0 * K[2] * px0 + K[1]
  const double epsi = -atan(K[1]);
```

# Model Predictive Control with Latency

Note we have to take the `100ms` latency into account, so instead of using the state as
churned out to as, we compute the state with the delay factored in using our kinematic model before feeding it to the object that will solve for what we should do next.

```cpp
  // current state must be in vehicle coordinates with the delay factored in
  // kinematic model is at play here
  // note that at current state at vehicle coordinates:
  // px, py, psi = 0.0, 0.0, 0.0
  // note that in vehicle coordinates it is going straight ahead the x-axis
  // which means position in vehicle's y-axis does not change
  // the steering angle is negative the given value as we have
  // as recall that during transformation we rotated all waypoints by -psi
  MPC mpc;

  const double dt = 0.1;

  const double current_px = 0.0 + v * dt;
  const double current_py = 0.0;
  const double current_psi = 0.0 + v * (-delta) / Lf * dt;
  const double current_v = v + a * dt;
  const double current_cte = cte + v * sin(epsi) * dt;
  const double current_epsi = epsi + v * (-delta) / Lf * dt;

  state << current_px, current_py, current_psi, current_v, current_cte, current_epsi;

  mpc.solve(state, K);

  cout << "steering angle:" << mpc.steer
       << "throttle:" << mpc.throttle << endl;

  //K is the coefficients of the 3rd order polynomial that estimates the road curve ahead
```
