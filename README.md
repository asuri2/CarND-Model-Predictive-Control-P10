# CarND-Model Predictive Control Project

Self-Driving Car Engineer Nanodegree Program

----------

### Introduction

This repository contains my solution to MPC project of SDC ND Term-2.

### The Model

#### 1.  States:-
    
    -   px : x-coordinate of current location of the vehicle in global map.
    -   py : y-coordinate of current location of the vehicle in global map.
    -   psi : Current orientation of the vehicle.
    -   v : Current velocity of the vehicle.
    -   cte : Cross track error, which is measured w.r.t desired position of the vehicle.
    -   epsi : Error in orientation of vehicle w.r.t desired orientation.

#### 2.  Actuations:-
    
    -   delta : Steering angle of the vehicle by which vehicle will turn. This angle is restricted between 25 to -25 degrees.
    -   a : Throttle or brake of the vehicle which accelerates or descelerates vehicle. The value of throttle is restricted between -1 to 1.

#### 3.  Kinematic Model:-
    
    -   px` = px + v * cos(psi) * dt
    -   py` = py + v * sin(psi) * dt
    -   psi` = psi + (v / Lf) * (- delta *dt)
    -   v` = v + (a * dt)
    -   cte` = cte + v * sin(epsi) * dt
    -   espi` = espi + v / Lf * -delta *dt

#### N & dt

**Values used in this project for N and dt are 10 and 0.1 respectively.**  Actually I have taken these values from Udacity QnA video. I initially started with multiple values of N ranging from 5 to 50 and dt from 0.05 to 0.5, but I found that by taking a large value of N and small value of dt was making my model very slow due to a lot of computation to process those number of points. So finally I decided to go with N as 10 and dt as 0.1 which worked well for me. And it makes sense since taking a low N will result in predicting for shorter future and taking it too high will result in predicting for longer future values which intern will result in complex and long running computations.

#### MPC Preprocessing

Before passing the values to polyfit I converted those to vehicle co-ordinate system which in-turn results in easier computations as vehicle's x and y are 0, 0. Since vehicle location will be treated as origin and orientation angle will be 0. 

    -   // shifting the points such that vehicle is at (0,0)
    -   double shift_x = ptsx[i]-px;
    -   double shift_y = ptsy[i]-py;
 
    -   // rotating the points such that vehicle's heading is 0 degrees
    -   ptsx[i] = (shift_x *cos(0-psi) - shift_y *sin(0-psi));
    -   ptsy[i] = (shift_x *sin(0-psi) + shift_y *cos(0-psi));

 Also, I used 3rd order polynomial for fitting as it can model road lanes properly and it gave good approximation. Taking higher order polynomial results in complex computations and taking lower order polynomial will not give proper approximation while calculating the minima.


#### Model Predictive Control with Latency
The code handles a latency of 100 milliseconds. In order to achieve this I have modified the initial state of the vehicle which is passed to the solver. I have taken the initial state and calculated the state of the vehicle 100 milliseconds into the future. This predicted state is then passed to the solver. Now the actuations returned by the solver are for the current time and can be directly applied to the vehicle. 
I have used the Kinematic model equations specified above to compute states after the period of latency. Then these predicted states are passed to model for optimizing.