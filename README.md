# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program.
Term2 final project: Model Predictive Control


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Implementation

The model uses 6 state variable vector to describe the car in current state:[px, py, psi, v, cte, epsi]
where
px: the car position of x-axis.
py: the car position of y-axis.
psi:the orientation of the vehicle in radiansconverted from the Unity format to the standard format expected in most mathemetical functions.
v: The current velocity of car in mph
cte: the cross track error
epsi: the error orentation compared to ideal situation.

In the main.cpp, upon every websocket message, the car position and waypoints are extracted from the message. Then subtract the car position[px, py] from the waypoint so that move the co-ordinate origin to car position. Then rotates the system by the current car orientation in psi so that in new co-ordinates, ideal car orentation is along the x-axis such that the CTE simply becomes the y-axis offset and counter-clock orentatioin is negative PSI(left turn) and clock-wise orentation PSI(right turn).

Here is the pusduo code:
  foreach waypoint in waypoints:
     waypoint_x_offset = waypoint[px] - car[px]
     waypoint_y_offset = waypoint[py] - car[py]
     
     // do the clock wise PSI rotation
     waypoint[px] = waypoint_x_offset * cos(0-psi) - waypoint_y_offset*sin(0-psi))
     waypoint[py] = waypoint_x_offset * sin(0-psi) + waypoint_y_offset*cos(0-psi))
  
