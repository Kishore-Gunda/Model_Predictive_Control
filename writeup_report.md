
# **Model Predictive Control** 

## Writeup

---

The steps in developing this project are the following:
* Initially MPC is implemented.
* Then the parameters are tuned manually to go around the loop.
* 100ms latency for actuator and action is added.
* Parameters and weights are tuned till proper driving is achieved.


### Files Submitted & Implementation Details

#### 1. Submission includes all required files 

My project includes the following files:
* src/main.cpp implements the main function to trigger the solver and send back actuator values to server
* src/MPC.cpp implements implements the solver and required dependencies.

#### 2. Model: The model used in the project is same as the one described in the classroom. 
The update equations for different parameters are as follows. 

![alt text](https://github.com/pkorivi/CarND-MPC-Project/blob/master/car_model.png)

State is comprised of the current position, orientation,velocity, cross track error, psi, currents teering angle and throttle values.

The project uses N = 10 and dt = 0.1. This choice has been taken after various tries (30:0.02, 30:0.05, 10:0.05, 10:0.1, 30:0.01 and so on). 
Initially 30:0.02 performed well but after adding the latency constraint, I started getting weird behavior in the predicted path, I turned to smaller N and larger dt as per suggestion. It improved performance and stability.

The waypoints are transformed into car's frame of reference as the waypoint display takes data in car frame and also the fitting curves are good with transformed points.


To battle latency and drive car smoothly, various techniques have been tried. Out of all I chose using the actuator values with 100ms delay from solver. It gave good results, but I would have preferred a solution which is mentioned in the lecture about considering the start state after a simulated time or limiting the acceleration and steering bundaries for initial 100ms. 

These ideas didn't give me a good performance. I would like to try these further while improving the computational and driving speed. 

The final [video](./result.mp4).

