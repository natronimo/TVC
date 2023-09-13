# Propulsive Lander Simulation
The Propulsive Lander Simulation is a Python based model of a thrust vector controlled rocket. The rocket will track to a reference position with Kalman Filtering and LQR control.

## Rocket Dynamics
The rocket has 13 states:
- x, y, z (position)
- v_x, v_y, v_z (velocity)
- theta_x, theta_y, theta_z (euler angles)
- omega_x, omega_y, omega_z (angular velocity)
- m (mass)

and 4 inputs: 
- T_x, T_y, T_z (body frame thrust)
- M_z (z-axis moment)

To derive the rocket dynamics, we will start with a general rigid body dynamics model of these 13 states. Note that the forces are in the inertial frame, the moments are in the body frame, and the moments of inertia are along the principal axes.

![image](https://github.com/natronimo/TVC/assets/123428083/65920fd8-7568-4e34-9ee4-d4129b4936ab)

To get the inertial frame forces (rotation matrix)
Next, we will substitute the inputs.

![image](https://github.com/natronimo/TVC/assets/123428083/e7f6ac6b-d724-46c5-b00d-22570029d96d)

## Simulation Block Diagram

![image](https://github.com/natronimo/TVC/assets/123428083/06092126-970a-4070-a211-a56f4fae2502)

The simulation begins with a state vector, **x**, an input vector, **u**, and a randomized disturbance force vector, **w**. These vectors are fed into the dxdt function which is numerically integrated through time to create the state vector at the next time step. The state vector is multiplied by the output matrix, **C**, and added to a randomized measurement noise vector, **v**, to create the output vector, **y**. The output vector is fed into a Kalman Filter, which uses the previous state estimate vector, **x_hat**, and the previous input vector to create the state estimate vector. The state estimate vector is subtracted by the reference state vector, **r_x**, multiplied by the LQR matrix, **K**, and added to the reference input vector, **r_u**, to create the input vector. The loop continues.

![output](https://github.com/natronimo/TVC/assets/123428083/cd65e841-fce6-46c0-80ca-718e3afd2271)

![image](https://github.com/natronimo/TVC/assets/123428083/c62b49c8-3fd3-49c1-95cf-f75fa2295240)

![image](https://github.com/natronimo/TVC/assets/123428083/e6b05c3c-0efa-4716-9149-9332ee6cb9fd)



## State Estimation
To account for process noise and measurement noise


## Control Law Design
Many of the most powerful methods of control are designed for linear systems. This includes the Linear-Quadratic Regulator (LQR), an optimal full state feedback control, and the method of choice for the propulsive lander simulation.
The rocket dynamics are nonlinear, but they can be linearized around a fixed point and represented as a state-space. A fixed point is a state where the system will remain in equilibrium when subject to a constant input. In other words, the time derivative of the state vector is an equally sized zero vector.
The rocket fixed point is:
- **v** = <0, 0, 0>
- **theta** = <0, 0, 0>
- **omega** = <0, 0, 0>
- **T** = <0, 0, m*g>
- M_z = 0

The fixed point is independent of the rocket position. Mass is dropped as a state for the purpose of the controller, as we are not trying to control it.

A linear system is represented in state-space form:


To find the system matrix **A**, the jacobian of **x_dot** with respect to **x** is calculated and then evaluated at the fixed point. To find the input matrix **B**, the jacobian of **x_dot** with respect to **u** is calculated and then evaluated at the fixed point.


The LQR matrix is found by solving a cost function, which takes the the system matrix, input matrix, state weight matrix, and input weight matrix as inputs. In the simulation, this calculation is done by the Python Control Systems Library.

The input matrix is a function of mass, which is constantly decreasing as a fuel is expended. This means that the LQR matrix needs to be recalculated at every time step.

## Visualization

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
