import random
import numpy as np
from numpy import linalg as LA
from scipy.spatial.transform import Rotation as rot
from scipy.integrate import solve_ivp
import control as ct

# simulation parameters
g = 9.81            # standard gravity (m/s^2)
time_step = 0.01    # simulation time step (s)
traj = [[0, 0, 0], [0, 0, 50], [10, 0, 50], [10, 10, 50], [0, 10, 50], [0, 0, 50], [0, 0, 0]]    # trajectory (m)
tol = 1             # position tolerance (m)
dist_std = 100      # disturbance force standard deviation (N)

# rocket parameters
m = 170             # initial mass (kg)
r = 0.2             # rocket radius (m)
L = 2               # rocket length (m)
r_T = 1             # thrust moment arm (m)
I_sp = 250          # specific impulse (s)
# principal moments of inertia (kg*m^2)
I_x = lambda m: (1/4)*m*r**2 + (1/12)*m*L**2
I_y = lambda m: (1/4)*m*r**2 + (1/12)*m*L**2
I_z = lambda m: (1/2)*m*r**2

# constraints
# thrust (N)
T_s_magLim = 300
T_z_magLim = 2500
# thrust rate (N/s)
T_s_rateLim = 600

# initialize arrays
x = np.array([[traj[0][0]], [traj[0][1]], [traj[0][2]], [0], [0], [0], [0], [0], [0], [0], [0], [0], [m]])
x_history = np.zeros((13, 1))
u_history = np.zeros((4, 1))
u_last = np.zeros((4, 1))

def dxdt(t, y):

    # velocity (m/s)
    v_x = y[3]
    v_y = y[4]
    v_z = y[5]
    # attitude (rad)
    theta_x = y[6]
    theta_y = y[7]
    theta_z = y[8]
    # angular velocity (rad/s)
    omega_x = y[9]
    omega_y = y[10]
    omega_z = y[11]
    # mass (kg)
    m = y[12]

    T_body = u[0:3]    # thrust vector body frame (N)
    rotm = rot.from_euler('zyx', [theta_z, theta_y, theta_x])    # rotation matrix
    T_inertial = np.matmul(rotm.as_matrix(), T_body)    # thrust vector inertial frame (N)
    T_mag = LA.norm(T_body)    # thrust magnitude (N)

    # intertial frame forces (N)
    F_x = T_inertial[0][0] + dist[0]
    F_y = T_inertial[1][0] + dist[1]
    F_z = T_inertial[2][0] - m*g
    # body frame moments (N*m)
    M_x = T_body[1][0]*r_T - dist[1]*r_dist
    M_y = -T_body[0][0]*r_T + dist[0]*r_dist
    M_z = u[3][0]
    # mass flow rate (kg/s)
    m_dot = -T_mag/(I_sp*g)

    # derivative of state wrt time
    return np.array([v_x,
                     v_y,
                     v_z,
                     F_x/m,
                     F_y/m,
                     F_z/m,
                     omega_x,
                     omega_y,
                     omega_z,
                     (M_x - (I_z(m) - I_y(m))*omega_y*omega_z)/I_x(m),
                     (M_y - (I_x(m) - I_z(m))*omega_x*omega_z)/I_y(m),
                     (M_z - (I_y(m) - I_x(m))*omega_x*omega_y)/I_z(m),
                     m_dot])

for pos in traj:

    ref = [[pos[0]], [pos[1]], [pos[2]], [0], [0], [0], [0], [0], [0], [0], [0], [0]]    # reference vector

    while LA.norm(ref - x[0:12]) > tol and x[2] > -0.1:

        m = x[12, 0]    # mass (kg)

        # state space matrices
        A = [[0, 0, 0, 1, 0, 0,  0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 1, 0,  0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 1,  0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0,  0, g, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, -g, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0,  0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 1],
             [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0]]
        B = [[          0,          0,   0,        0],
             [          0,          0,   0,        0],
             [          0,          0,   0,        0],
             [        1/m,          0,   0,        0],
             [          0,        1/m,   0,        0],
             [          0,          0, 1/m,        0],
             [          0,          0,   0,        0],
             [          0,          0,   0,        0],
             [          0,          0,   0,        0],
             [          0, r_T/I_x(m),   0,        0],
             [-r_T/I_y(m),          0,   0,        0],
             [          0,          0,   0, 1/I_z(m)]]
        
        Q = np.diag([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])    # state weights
        R = np.diag([0.01, 0.01, 0.01, 0.01])    # input weights
        K, S, E = ct.lqr(A, B, Q, R)    # feedback matrix
        u = np.matmul(K, ref - x[0:12]) + [[0], [0], [m*g], [0]]    # input

        # constraints
        u[0, 0] = max(-T_s_magLim, min(T_s_magLim, u[0, 0]))
        u[1, 0] = max(-T_s_magLim, min(T_s_magLim, u[1, 0]))
        u[2, 0] = max(0, min(T_z_magLim, u[2, 0]))
        u[0, 0] = min(u[0, 0] - u_last[0, 0], T_s_rateLim*time_step) + u_last[0, 0]
        u[0, 0] = max(u[0, 0] - u_last[0, 0], -T_s_rateLim*time_step) + u_last[0, 0]
        u[1, 0] = min(u[1, 0] - u_last[1, 0], T_s_rateLim*time_step) + u_last[1, 0]
        u[1, 0] = max(u[1, 0] - u_last[1, 0], -T_s_rateLim*time_step) + u_last[1, 0]

        # data capture
        x_history = np.append(x_history, x, 1)
        u_history = np.append(u_history, u, 1)
        u_last = u

        # disturbance force
        dist = [random.gauss(0, dist_std), random.gauss(0, dist_std)]
        r_dist = random.triangular(-L/2, L/2)
        
        # integrate state through time
        sol = solve_ivp(dxdt, [0, time_step], np.transpose(x)[0], t_eval=[time_step])
        x = sol.y

np.savetxt("x_history.out", x_history)
np.savetxt("u_history.out", u_history)
