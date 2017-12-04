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

In the main.cpp, upon every websocket message, the car position and waypoints are extracted from the message. Then subtract the car position[px, py] from the waypoint so that move the co-ordinate origin to car position. Then rotates the system by the current car orientation in psi so that in new co-ordinates, ideal desired car orentation is along the x-axis such that the CTE simply becomes the y-axis offset and counter-clock orentatioin is negative PSI(left turn) and clock-wise orentation PSI(right turn).

Here is the pusduo code:

* Step 1: co-ordinate translation
     // do the clock wise PSI rotation
     
     
     foreach waypoint in waypoints{
       waypoint_x_offset = waypoint[px] - car[px]
       waypoint_y_offset = waypoint[py] - car[py]
       waypoint[px] = waypoint_x_offset * cos(0-psi) - waypoint_y_offset*sin(0-psi))
       waypoint[py] = waypoint_x_offset * sin(0-psi) + waypoint_y_offset*cos(0-psi))
     }
     
     
* Step 2: Polyfit the waypoints to find the coeffs
   Simply call the polyfit witht the waypoints from step1 to fit them into polynomial of order of 3.
   
* Step 3: Calcuate the CTE and epsi
   The CTE becomes straightforward by simply apply the polyeval at origin of zero where car is positioned, i.e. polyeval(coeffs, 0)
   The epsi, error orentation is the angle of atan of first derivative of the polynomial at car position, i.e. new origion.
   first derivative(fd1) = coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs * px * px
   since px in new coordinate is of zero, above becomes: fd1 = coeffs[1]
   so the error orentation is epsi = psi - atan(coeffs[1]) = - atan(coeff[1] since PSI desired is of zero to drive along x-axis.
   
* step 4: calcuate the steering angle and throttle using the MPC, such as
      vars = mpc.Solve(state, coeffs)
      
* step 5: normalize the angle to  [ -25, 25] in radias and display the waypoints and projected path to simulator. The actuator is normalized between [-1, 1] when MPC:Solve. no further action is needed.

**FG_eval**: The model update constraint is govened by following fomula in Class FG_eval:

* Cost fg[0] has the cost of various conditions:
     1. CTE of N steps
     2. epsi of N steps
     3. speed offset to reference speed(100MPH in the final submission)
     4. orentation error
     5. actuation 
     6. difference of current orentation and previous orentation error
     7. difference of current actuation and previous actuation
* model constraint update:


