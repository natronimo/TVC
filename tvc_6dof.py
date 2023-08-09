import numpy as np
from numpy import linalg as LA
from scipy.spatial.transform import Rotation as rot
from scipy.integrate import solve_ivp
import control as ct

g = 9.81
r_T = 1
L = 2
r = 0.2
I_sp = 250
time_step = 0.01

x = -10
y = 10
z = 50
v_x = 0
v_y = 0
v_z = 0
theta_x = 0
theta_y = 0
theta_z = 0
omega_x = 0
omega_y = 0
omega_z = 0
m = 170

I_x = lambda m: (1/4)*m*r**2 + (1/12)*m*L**2
I_y = lambda m: (1/4)*m*r**2 + (1/12)*m*L**2
I_z = lambda m: (1/2)*m*r**2

x_ref = 0
y_ref = 0
z_ref = 0

T_s_lim = 300
T_z_lim = 2500
M_z_lim = 100

x = np.array([[x], [y], [z], [v_x], [v_y], [v_z], [theta_x], [theta_y], [theta_z], [omega_x], [omega_y], [omega_z], [m]])
ref = [[x_ref], [y_ref], [z_ref], [0], [0], [0], [0], [0], [0], [0], [0], [0]]
x_history = np.zeros((13, 1))
u_history = np.zeros((4, 1))

def dxdt(t, y):

    v_x = y[3]
    v_y = y[4]
    v_z = y[5]
    theta_x = y[6]
    theta_y = y[7]
    theta_z = y[8]
    omega_x = y[9]
    omega_y = y[10]
    omega_z = y[11]
    m = y[12]

    rotm = rot.from_euler('zyx', [theta_z, theta_y, theta_x])
    T_body = u[0:3]
    T_inertial = np.matmul(rotm.as_matrix(), T_body)
    T_mag = LA.norm(T_body)

    F_x = T_inertial[0, 0]
    F_y = T_inertial[1, 0]
    F_z = T_inertial[2, 0] - m*g
    M_x = T_body[1, 0]*r_T
    M_y = -T_body[0, 0]*r_T
    M_z = u[3, 0]
    m_dot = -T_mag/(I_sp*g)

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

while x[2] > 0:

    m = x[12, 0]

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
    Q = np.diag([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    R = np.diag([0.01, 0.01, 0.01, 0.01])
    K, S, E = ct.lqr(A, B, Q, R)
    u = np.matmul(K, ref - x[0:12]) + [[0], [0], [m*g], [0]]

    u[0, 0] = max(-T_s_lim, min(T_s_lim, u[0, 0]))
    u[1, 0] = max(-T_s_lim, min(T_s_lim, u[1, 0]))
    u[2, 0] = max(0, min(T_z_lim, u[2, 0]))
    u[3, 0] = max(-M_z_lim, min(M_z_lim, u[3, 0]))

    x_history = np.append(x_history, x, 1)
    u_history = np.append(u_history, u, 1)
    
    sol = solve_ivp(dxdt, [0, time_step], np.transpose(x)[0], t_eval=[time_step])
    x = sol.y
