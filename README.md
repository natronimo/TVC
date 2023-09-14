# Propulsive Lander Simulation
The Propulsive Lander Simulation is a Python based six degrees of freedom variable mass thrust vector controlled rocket simulation. The rocket uses LQR control and Kalman Filter state estimation to track to a given reference position. The rocket mass, principal moments of inertia, thrust moment arm, specific impulse, and thrust constraints are all modifiable, as well as the weight matrices for the LQR and Kalman Filter. "tvc_lqr" runs the simulation and saves a text file of the rocket state at every time step, which is used by "flightgear_interface" to render the simulation in FlightGear Flight Simulator.

## Rocket Dynamics
The rocket has 13 states:
- position (x, y, z)
- velocity (v<sub>x</sub>, v<sub>y</sub>, v<sub>z</sub>)
- euler angles (θ<sub>x</sub>, θ<sub>y</sub>, θ<sub>z</sub>)
- angular velocity (ω<sub>x</sub>, ω<sub>y</sub>, ω<sub>z</sub>)
- mass (m)

and 4 inputs: 
- body frame thrust (T<sub>x</sub>, T<sub>y</sub>, T<sub>z</sub>)
- z-axis moment (M<sub>z</sub>)

We will start with a general rigid body dynamics model of these 13 states. Note that the forces are in the inertial frame, the moments are in the body frame, and the moments of inertia are about the principal axes.

![image](https://github.com/natronimo/TVC/assets/123428083/65920fd8-7568-4e34-9ee4-d4129b4936ab)

The inertial frame thrust vector is the product of the rotation matrix, a function of the euler angles, and the body frame thrust vector.

The rocket dynamics model is found by substituting the forces and moments acting on the rocket into the general model.

![image](https://github.com/natronimo/TVC/assets/123428083/e7f6ac6b-d724-46c5-b00d-22570029d96d)

The state vector will henceforth be referred to as **x** and the time derivative of the state vector as **ẋ**.

## Simulation Block Diagram
![image](https://github.com/natronimo/TVC/assets/123428083/06092126-970a-4070-a211-a56f4fae2502)

The simulation begins with a state vector, an input vector, and a randomized disturbance force vector, **w**. These vectors are fed into the dxdt function which is numerically integrated through time to create the state vector at the next time step. The state vector is multiplied by the output matrix and added to a randomized measurement noise vector, **v**, to create the output vector. The output vector is fed into a Kalman Filter, which uses the previous state estimate vector and the previous input vector to create the state estimate vector. The state estimate vector is subtracted by the reference state vector, multiplied by the LQR matrix, and added to the reference input vector to create the input vector. The loop continues.

![image](https://github.com/natronimo/TVC/assets/123428083/3262b7c8-aa8b-40af-8585-afff4f870fc8)
![image](https://github.com/natronimo/TVC/assets/123428083/f01fc739-cd5a-42cc-b785-fac83be6897d)

In the following sections, we will derive the above matrices.

## Control Law Design
The most powerful techniques in control theory are developed for linear dynamical systems. This includes the Linear-Quadratic Regulator (LQR), which is the technique of choice for the propulsive lander simulation. A linear dynamical system is of the form:

![image](https://github.com/natronimo/TVC/assets/123428083/2aef6a0d-afec-486b-bc27-de61871bdd85)

The rocket dynamics are nonlinear, but they can be linearized around a fixed point to derive the system matrix, **A**, and the input matrix, **B**. A fixed point is a state where the system will remain in equilibrium when subject to a constant input. In other words, the time derivative of the state vector is an equally sized zero vector.
The state and input for the rocket fixed point are:
- **v** = (0, 0, 0)
- **θ** = (0, 0, 0)
- **ω** = (0, 0, 0)
- **T** = (0, 0, mg)
- M<sub>z</sub> = 0

The fixed point is independent of the rocket position. Mass is dropped as a state for the purpose of the controller, as we are not trying to control it.

To find the system matrix **A**, the jacobian of **ẋ** with respect to **x** is computed and then evaluated at the fixed point. To find the input matrix **B**, the jacobian of **ẋ** with respect to **u** is computed and then evaluated at the fixed point.

![image](https://github.com/natronimo/TVC/assets/123428083/b17e75e2-8bdc-4f16-8638-f7cfc28f404d)

The LQR feedback matrix is found by solving a cost function, which takes as inputs the system matrix, input matrix, state weight matrix, and input weight matrix. In the simulation, the LQR feedback matrix is computed by the Python Control Systems Library.

The input matrix is a function of mass, which is variable. This means that the LQR matrix needs to be recalculated at every time step with the new mass.

## State Estimation


## Visualization
1. Copy and paste the MATLAB "HL20" folder
    - from  C:\Program Files\MATLAB\R2023a\toolbox\aero\animation
    - to    C:\Program Files\FlightGear 2020.3\data\Aircraft
2. Within the FlightGear "HL20" folder under "Models," replace the existing "HL20.ac" file with the one from the repo.
3. Edit the "tvc_lqr.py" parameters as needed.
4. Run "tvc_lqr.py" to create the "x_history.out" file.
5. Run "runfg.bat" to open FlightGear with the proper configurations.
6. Run "flightgear_interface.py" and watch the rocket fly.
