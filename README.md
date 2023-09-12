# Propulsive Lander Simulation
The Propulsive Lander Simulation is a Python based model of a thrust vector controlled rocket. The rocket will track to a reference position with Kalman Filtering and LQR control.

## Rocket Dynamics
The rocket has 13 states: position vector, velocity vector, euler angle vector, angular velocity vector, mass; and 4 inputs: body frame thrust vector, z-axis moment. To derive the dynamics, we will start with a general rigid body dynamics model of these 13 states. Note that the forces are in the inertial frame, the moments are in the body frame, and the moments of inertia are along the principal axes.

![image](https://github.com/natronimo/TVC/assets/123428083/7581e20c-3aba-4646-aebd-4bea45ead43d)

Next, we will substitute the inputs into the above equation.

![image](https://github.com/natronimo/TVC/assets/123428083/6331b88f-099a-455f-8a7d-087520818810)

## Simulation Block Diagram
The simulation begins with a state vector, **x**, an input vector, **u**, and a randomized disturbance force vector, **w**. These vectors are fed into the dxdt function which is numerically integrated through time to create the state vector at the next time step. The state vector is multiplied by the output matrix, **C**, and added to a randomized measurement noise vector, **v**, to create the output vector, **y**. The output vector is fed into a Kalman Filter, which uses the previous state estimate vector, **x_hat**, and the previous input vector to create the state estimate vector. The state estimate vector is subtracted by the reference state vector, **r_x**, multiplied by the LQR matrix, **K**, and added to the reference input vector, **r_u**, to create the input vector. The loop continues.

![image](https://github.com/natronimo/TVC/assets/123428083/5192a821-006b-4d5c-80e5-5707d60727db)

## Control Law Design
The most powerful methods of control are designed for linear systems. The rocket dynamics are highly nonlinear, but they can be linearized around a fixed point to utilize such methods as optimal control. A fixed point, for our purposes, is a state where the system will remain in equlibrium when subject to a constant input. The rocket's fixed point is in the vertical orientation with the body frame thrust vector represented as <0, 0, mg>. This presents a problem because mass is constantly decreasing as fuel is expended.
To overcome this, we will everywhere substitute mg for T and make the linearized model a function of the rocket's mass.

![image](https://github.com/natronimo/TVC/assets/123428083/30502b0f-4da0-4490-9e54-980caa951bc8)

## State Estimation
To account for 

## Visualization


## How to Use
Video
https://cpslo-my.sharepoint.com/:v:/g/personal/nthoma14_calpoly_edu/ETfcNDsdT2JLjz27S7goXX0BPS6JHFs33rpMhSr27MmpSA

1. Copy and paste the MATLAB "HL20" folder
    - from  "C:\Program Files\MATLAB\R2023a\toolbox\aero\animation"
    - to    "C:\Program Files\FlightGear 2020.3\data\Aircraft."

2. Within the FlightGear "HL20" folder under "Models," replace the existing "HL20.ac" file with the one from the repo.

3. Edit the "tvc_6dof.py" parameters as needed.

4. Run "tvc_6dof.py" and ensure the "x_history.out" file has been created.

5. Run "runfg.bat" to open FlightGear with the proper configuration.

6. Run "flightgear_interface.py" and watch the rocket fly.
