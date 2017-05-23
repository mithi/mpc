# INTRODUCTION

This is my turn-in code for one of the project in partial fulfillment of the requirements for Udacity's self-driving car Nanodegree program.

In this project, I have implemented a Model Predictive Control software pipeline to drive a car around a track in a simulator. There is a 100 millisecond latency between actuation commands on top of the connection latency.

Please view my screen recording of using my code for the vehicle to drive around the track for
several laps:
- https://www.youtube.com/watch?v=75ylhM0QsXQ&feature=youtu.be
The MPC trajectory path is displayed in green, and the polynomial fitted reference path in yellow.

According to wikipedia:
> Model predictive controllers rely on dynamic models of the process. The main advantage of MPC is the fact that it allows the current timeslot to be optimized, while keeping future timeslots in account. This is achieved by optimizing a finite time-horizon, but only implementing the current timeslot. MPC has the ability to anticipate future events and can take control actions accordingly.

In my own words, the MPC method can anticipate future events because we have an idea of what is probably gonna happen if we choose to do what we are going to do. This is because we have a model of how things work in our world (like physics for example). We can anticipate future events based on our current plan of action and also anticipate our next plan of action based on the result of the current plan. Here is a nice series of videos that give a nice overview about the concept behind MPC as well as an in depth discussion:
- https://www.youtube.com/watch?v=4kCcXGDvjU8&list=PLs7mcKy_nInFEpygo_VrqDFCsQVnGaoy-

- For more information, you can check the initial project repo from Udacity here:
-- https://github.com/udacity/CarND-MPC-Project
- The simulator used can be downloaded here:
-- https://github.com/udacity/self-driving-car-sim/releases
- The flow of the code is largely based on Udacity's sample code and quizzes
--


# THE MODEL

**The state variables for the vehicle are the following:**

## `px`
 - the current location in the x-axis of an arbitrary global map coordinate system
## `py`
 - the current location in the x-axis of an arbitrary global map coordinate system
## `psi`
 - the current orientation / heading of the vehicle
## `v`
 - the current velocity/speed of the vehicle

This is given to us by the simulation every time we ask for it. In addition to this,
we are also given a series of `waypoints` which are points with respect to
an arbitrary global map coordinate system we can use to fit a polynomial which is a function
that estimates the curve of the road ahead. It is known that a 3rd degree polynomial can
be a good estimate of most road curves. The polynomial function is with respect to the vehicle's
local coordinate.

**We can compute for the errors which is the difference between our desired position and heading
and our actual position and heading:**

## `cte`
 - this is the cross track error which is the difference between our desired position and actual position. We can use our fitted polynomial at point px = 0 to get the position where we should
 currently be.

## `epsi`
 - this is the orientation error which is the difference between our desired heading and actual heading. Our desired orientation is the heading tangent to our road curve. This can be computed using an arctan to the derivative of the fitted polynomial function at point px = 0 to get the angle
 to which we should be heading.

From this we use Model Predictive Control to give what what is our best course
of action. There are two modes of *actuation* we can use to control our vehicle

## `delta`
  - this is the steering value which represents the angle of which we turn our vehicle, which I suppose is the angle of the vehicle's tires. The angle is restricted to be between -25 and 25 degrees but is mapped to the values between -1 and 1. **In my code, I have restricted this to only be between -0.75 and 0.75** as I which to be conservative as opposed to aggressive in our steering.
  Since we can plan ahead, we shouldn't be needing to suddenly steer a large angle.

## `a`
  - this is the *throttle* or *brake* value which represents the acceleration or deceleration of our vehicle. In an actual vehicle, this is controlled by the brake pedal. The simulator expects values between -1 and 1. Negative values represents braking and positive values represents throttle.
  **In my code, I have restricted the range to only be between -0.5 and 1** as ideally we shouldn't be suddenly pressing the brakes all of a sudden since we are able to plan ahead.

So based on a Physics, here is a simplified version of how the world (with our vehicle in it) works.
We call it our *kinematic* model.

# Time Step Length and Frequency

# Cost Function and Penalty Weights

# Polynomial Fitting and MPC Preprocessing

# Model Predictive Control with Latency
