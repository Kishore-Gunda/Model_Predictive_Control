
# **Model Predictive Control** 

## Writeup

---

The project is developed as below:
* First the MPC is implemented.
* After a lot of trail and error the parameters are tuned manually to go follow the track.
* A latency of 100ms is added for the action and actuator.
* Till the proper drive is achieved the parameters and weights are tuned.




### Files involved and Implementation Details:

#### 1. Files involved.

The below files are created/modify to implement the project.
* main.cpp this function trigger the solver and actuator values are send back to the server.
* MPC.cpp implements the dependencies and also implements the solver.


#### 2. Model: The same model as described in the class room is used in this implementation. 
For the dirrerent parameters the update equatins as follows. 

![alt text](https://github.com/Kishore-Gunda/Model_Predictive_Control/blob/master/model.png)

The current position, throttle value, currecnt steering values, psi, cross track error, velocity, orientation, comprises the State.


The N and dt values are as follows:
N =10 and
dt = 0.1
the various combinations of dt an N are tries like (.02, 20), (.03, 30), (.05, 10), (.01, 30), (.05, 10) ... 
Some pairs like (.02, 30), worked out well but after some parametrs are considered like latency the predected path been changed drastically.
Later as per the sugession the tried to tune the N and Dt to smaller and larger values and that improved the stability and performance.

The cars's frame of reference are from the waypoints, the fittings curves are good with teh transformed points.

To fix the latency issue and the bring an smooth curve different techniques has been tried and out of all I have selected to choose the actuator value with a delay of 100ms from the solver.It gives and good result.

But I will prefer the solution discussed in lectures about considerng the start state after the similated time or accelaration limitation steering boundries for first 100ms, as these ideas unfortunaely did not give me the expected output, some time later I will try to improve the computations and speed of the drive 


Thanks.

