# Propulsive Lander Simulation
The Propulsive Lander Simulation is a Python based six degrees of freedom variable mass thrust vector controlled rocket simulation. The rocket uses LQR control and Kalman Filter state estimation to track to a given reference position. The rocket mass, principal moments of inertia, thrust moment arm, specific impulse, and thrust constraints are all modifiable, as well as the weight matrices for the LQR and Kalman Filter. "tvc_lqr.py" runs the simulation and saves a text file of the rocket state at each time step, which is used by "flightgear_interface.py" to render the simulation in FlightGear Flight Simulator.

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

![image](https://github.com/natronimo/TVC/assets/123428083/ef76e3d4-d57f-4017-9a99-2a99e688a76c)

The rocket dynamics model is found by substituting the forces and moments acting on the rocket into the general model.

![image](https://github.com/natronimo/TVC/assets/123428083/e7f6ac6b-d724-46c5-b00d-22570029d96d)

## Simulation Block Diagram

![image](https://github.com/natronimo/TVC/assets/123428083/a94c63f2-d154-476a-966c-919c594e87e6)

The simulation begins with a state vector **x**, an input vector **u**, and a randomized disturbance force vector **w**. These vectors are fed into the dxdt function which is numerically integrated through time to create the state vector **x** at the next time step. The state vector **x** is the position, velocity, euler angles, angular velocity, and mass.

The state vector **x** is multiplied by the output matrix **C** and added to a randomized measurement noise vector **v**, to create the output vector **y**. The output vector **y** is the state vector that the rocket observes from the sensors.

![image](https://github.com/natronimo/TVC/assets/123428083/33a8caaa-c08e-40cf-95ac-3334449677e6)

The output vector **y** is fed into a Kalman Filter, which uses the previous state estimate vector **x̂** and the previous input vector **u** to create the state estimate vector **x̂**. The state estimate vector **x̂** is an estimate of the true state vector **x**, accounting for the sensor noise **v**, based on a model of the rocket dynamics and disturbace forces.

![image](https://github.com/natronimo/TVC/assets/123428083/4f1d0a1b-9206-4411-b5e6-49999b06c5f8)

The state estimate vector **x̂** is subtracted by the reference state vector **r**<sub>x</sub>, multiplied by the LQR matrix **K**, and added to the reference input vector **r**<sub>u</sub> to create the input vector **u**. The input vector **u** is the body frame thrust and z-axis moment.

![image](https://github.com/natronimo/TVC/assets/123428083/328a033d-048a-4b67-9500-546704b18e6b)

![image](https://github.com/natronimo/TVC/assets/123428083/cbe8823e-4036-4633-b1a0-cd3ffc658f60)

The optimal forms of the regulator matrix and estimator matrix are known as the LQR matrix and Kalman matrix, respectively. In the following sections, will derive the above matrices.

## Regulator Design
The most porful techniques in control theory are developed for linear dynamical systems. This includes the Linear-Quadratic Regulator (LQR), which is the technique of choice for the propulsive lander simulation. A continuous linear dynamical system is of the form:

![image](https://github.com/natronimo/TVC/assets/123428083/2aef6a0d-afec-486b-bc27-de61871bdd85)

The rocket dynamics are nonlinear, but they can be linearized around a fixed point to derive the system matrix, **A**, and the input matrix, **B**. A fixed point is a state where the system will remain in equilibrium when subject to a constant input. In other words, the time derivative of the state vector is an equally sized zero vector.
The state and input for the rocket fixed point are:
- **v** = (0, 0, 0) m/s
- **θ** = (0, 0, 0) rad
- **ω** = (0, 0, 0) rad/s
- **T** = (0, 0, mg) N
- M<sub>z</sub> = 0 N*m

The fixed point is independent of the rocket position. Mass is dropped as a state for the purpose of the controller, as  are not trying to control it.

To derive the system matrix **A**, the jacobian of **ẋ** with respect to **x** is computed and then evaluated at the fixed point. To derive the input matrix **B**, the jacobian of **ẋ** with respect to **u** is computed and then evaluated at the fixed point.

![image](https://github.com/natronimo/TVC/assets/123428083/b17e75e2-8bdc-4f16-8638-f7cfc28f404d)

The LQR matrix is found by solving a cost function, which takes as inputs the system matrix, input matrix, state weight matrix, and input weight matrix. In the simulation, the LQR matrix is computed by the Python Control Systems Library. The input matrix is a function of mass, which is variable over the course of the simulation. This means that the LQR matrix needs to be recomputed at each time step with the new mass.

## Estimator Design
Kalman filtering is best practiced with a discrete linear model of the system. A discrete linear dynamical system is of the form:

![image](https://github.com/natronimo/TVC/assets/123428083/e76879f2-f1a0-4384-874d-dc86967bdae3)

To derive the discrete model, we start with the continuous model. The integral of each state with respect to time is evaluated from zero to the time step (the time from *k* to *k*+1) with initial value the state at time *k*.

![rocket_linearized_dynamics_discrete](https://github.com/natronimo/TVC/assets/123428083/a7144e90-4d59-43b4-9318-cbe47026f5d3)

The Kalman matrix is found by solving a cost function, which takes as inputs the system matrix, process noise matrix, output matrix, process noise covariance matrix, and the sensor noise covariance matrix. In the simulation, the Kalman matrix is computed by the Python Control Systems Library. All of the inputs are constant over the course of the simulation. This means that the Kalman matrix can be computed at the beginning of the simulation and used repeatedly throughout.

## Visualization
"tvc_lqr" will plot the trajectory of the rocket with the accompanying output and state estimate at the end of the simulation.

![image](https://github.com/natronimo/TVC/assets/123428083/73aeebdc-6e99-428e-ab36-108f6e59850f)

In addition to this, the simulation can be rendered in FlightGear Flight Simulator by following the below steps.

1. Copy and paste the MATLAB "HL20" folder
    - from  C:\Program Files\MATLAB\R2023a\toolbox\aero\animation
    - to    C:\Program Files\FlightGear 2020.3\data\Aircraft
2. Within the FlightGear "HL20" folder under "Models," replace the existing "HL20.ac" file with the one from the repo.
3. Edit the "tvc_lqr.py" parameters as needed.
4. Run "tvc_lqr.py" to create the "x_history.out" file.
5. Run "runfg.bat" to open FlightGear with the proper configurations.
6. Run "flightgear_interface.py" and watch the rocket fly.

![image](https://github.com/natronimo/TVC/assets/123428083/f251fd27-b43f-4cd9-8333-4fea34870105)
