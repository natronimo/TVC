import numpy as np
from scipy.integrate import solve_ivp
import control as ct
import matplotlib.pyplot as plt

# rocket parameters
m = 170               # initial mass (kg)
r = 0.2               # rocket radius (m)
L = 2                 # rocket length (m)
r_T = 1               # thrust moment arm (m)
I_sp = 250            # specific impulse (s)
T_s_magLim = 300      # x y axes thrust limit (N)
T_z_magLim = 2500     # z axis thrust limit (N)
T_s_rateLim = 1000    # x y axes thrust rate limit (N/s)
# principal moments of inertia (kg*m^2)
I_x = lambda m: (1/4)*m*r**2 + (1/12)*m*L**2
I_y = lambda m: (1/4)*m*r**2 + (1/12)*m*L**2
I_z = lambda m: (1/2)*m*r**2

# regulator parameters
Q = np.eye(12)        # state weight matrix
R = 0.01*np.eye(4)    # input weight matrix

# estimator parameters
QN = np.eye(12)       # process noise covariance matrix
RN = np.eye(12)       # measurement noise covariance matrix

# simulation parameters
g = 9.81              # gravitational acceleration (m/s^2)
Ts = 0.01             # time step (s)
N = 1000              # total steps
x = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [m]])    # initial state vector
u = np.array([[0], [0], [m*g], [0]])                                               # initial input vector
ref = np.array([[0], [0], [10], [0], [0], [0], [0], [0], [0], [0], [0], [0]])      # reference state vector
v_std = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])      # measurement noise standard deviation matrix

# system matrix
A = np.array([[1, 0, 0, Ts,  0,  0,           0, -(Ts**2*g)/2, 0,           0, -(Ts**3*g)/6,  0],
              [0, 1, 0,  0, Ts,  0, (Ts**2*g)/2,            0, 0, (Ts**3*g)/6,            0,  0],
              [0, 0, 1,  0,  0, Ts,           0,            0, 0,           0,            0,  0],
              [0, 0, 0,  1,  0,  0,           0,        -Ts*g, 0,           0, -(Ts**2*g)/2,  0],
              [0, 0, 0,  0,  1,  0,        Ts*g,            0, 0, (Ts**2*g)/2,            0,  0],
              [0, 0, 0,  0,  0,  1,           0,            0, 0,           0,            0,  0],
              [0, 0, 0,  0,  0,  0,           1,            0, 0,          Ts,            0,  0],
              [0, 0, 0,  0,  0,  0,           0,            1, 0,           0,           Ts,  0],
              [0, 0, 0,  0,  0,  0,           0,            0, 1,           0,            0, Ts],
              [0, 0, 0,  0,  0,  0,           0,            0, 0,           1,            0,  0],
              [0, 0, 0,  0,  0,  0,           0,            0, 0,           0,            1,  0],
              [0, 0, 0,  0,  0,  0,           0,            0, 0,           0,            0,  1]])
# input matrix
B = lambda m: np.array([[(g*r_T*Ts**4)/(24*I_y(m)) + Ts**2/(2*m),                                       0,           0,                0],
                        [                                      0, (g*r_T*Ts**4)/(24*I_x(m)) + Ts**2/(2*m),           0,                0],
                        [                                      0,                                       0, Ts**2/(2*m),                0],
                        [        (g*r_T*Ts**3)/(6*I_y(m)) + Ts/m,                                       0,           0,                0],
                        [                                      0,         (g*r_T*Ts**3)/(6*I_x(m)) + Ts/m,           0,                0],
                        [                                      0,                                       0,        Ts/m,                0],
                        [                                      0,                  (Ts**2*r_T)/(2*I_x(m)),           0,                0],
                        [                -(Ts**2*r_T)/(2*I_y(m)),                                       0,           0,                0],
                        [                                      0,                                       0,           0, Ts**2/(2*I_y(m))],
                        [                                      0,                         (Ts*r_T)/I_x(m),           0,                0],
                        [                       -(Ts*r_T)/I_y(m),                                       0,           0,                0],
                        [                                      0,                                       0,           0,        Ts/I_y(m)]])
C = np.eye(12)        # output matrix

# initialize arrays
x_history = np.empty((13, N))
y_history = np.empty((12, N))
xe_history = np.empty((12, N))
u_history = np.empty((4, N))
xe_last = x[0:12]
P_last = np.ones((12, 12))
u_last = u

def dxdt(t, y):

    # velocity (m/s)
    v_x = y[3]
    v_y = y[4]
    v_z = y[5]
    # euler angles (rad)
    phi = y[6]
    theta = y[7]
    psi = y[8]
    # angular velocity (rad/s)
    omega_x = y[9]
    omega_y = y[10]
    omega_z = y[11]
    # mass (kg)
    m = y[12]

    # body frame to inertial frame rotation matrix
    C_ib = np.array([[                                      np.cos(psi)*np.cos(theta),                                       np.cos(theta)*np.sin(psi),            -np.sin(theta)],
                     [np.cos(psi)*np.sin(phi)*np.sin(theta) - np.cos(phi)*np.sin(psi), np.cos(phi)*np.cos(psi) + np.sin(phi)*np.sin(psi)*np.sin(theta), np.cos(theta)*np.sin(phi)],
                     [np.sin(phi)*np.sin(psi) + np.cos(phi)*np.cos(psi)*np.sin(theta), np.cos(phi)*np.sin(psi)*np.sin(theta) - np.cos(psi)*np.sin(phi), np.cos(phi)*np.cos(theta)]])
    # angular velocity to euler angle rates rotation matrix
    C_ib_rate = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                          [0,               np.cos(phi),              -np.sin(phi)],
                          [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

    T_body = u[0:3]                   # thrust body frame vector (N)
    T_inertial = C_ib @ T_body        # thrust inertial frame vector (N)
    T_mag = np.linalg.norm(T_body)    # thrust magnitude (N)

    # intertial frame forces (N)
    F_x = T_inertial[0, 0]
    F_y = T_inertial[1, 0]
    F_z = T_inertial[2, 0] - m*g
    # euler angle rates
    E_dot = C_ib_rate @ np.array([[omega_x], [omega_y], [omega_z]])
    phi_dot = E_dot[0, 0]
    theta_dot = E_dot[1, 0]
    psi_dot = E_dot[2, 0]
    # body frame moments (N*m)
    M_x = T_body[1, 0]*r_T
    M_y = -T_body[0, 0]*r_T
    M_z = u[3, 0]
    # mass flow rate (kg/s)
    m_dot = -T_mag/(I_sp*g)

    # derivative of state vector wrt time
    return np.array([v_x,
                     v_y,
                     v_z,
                     F_x/m,
                     F_y/m,
                     F_z/m,
                     phi_dot,
                     theta_dot,
                     psi_dot,
                     (M_x - (I_z(m) - I_y(m))*omega_y*omega_z)/I_x(m),
                     (M_y - (I_x(m) - I_z(m))*omega_x*omega_z)/I_y(m),
                     (M_z - (I_y(m) - I_x(m))*omega_x*omega_y)/I_z(m),
                     m_dot])

for t in range(N):

    # integrate state vector through time
    sol = solve_ivp(dxdt, [0, Ts], np.transpose(x)[0], t_eval=[Ts])
    x = sol.y                                                 # state vector

    v = v_std @ np.random.standard_normal((12, 1))            # measurement noise vector
    y = C @ x[0:12] + v                                       # output vector

    xe = A @ xe_last + B(m) @ u_last                          # state estimate vector a priori
    P = A @ P_last @ A.T + QN                                 # estimate error covariance matrix a priori
    Ke = P @ C.T @ np.linalg.inv(C @ P @ C.T + RN)            # Kalman gain matrix
    xe = xe + Ke @ (y - C @ xe)                               # state estimate vector a posteriori
    P = (np.eye(12) - Ke @ C) @ P                             # estimate error covariance matrix a posteriori

    m = x[12, 0]                                              # mass (kg)
    Kr = ct.dlqr(A, B(m), Q, R)[0]                            # LQR gain matrix
    u = Kr @ (ref - xe) + np.array([[0], [0], [m*g], [0]])    # input vector

    # constrain input
    u[0:2] = np.clip(u[0:2], -T_s_magLim, T_s_magLim)
    u[2] = np.clip(u[2], 0, T_z_magLim)
    u[0:2] = np.clip(u[0:2] - u_last[0:2], -T_s_rateLim*Ts, T_s_rateLim*Ts) + u_last[0:2]

    # capture data
    x_history[:, t] = x[:, 0]
    y_history[:, t] = y[:, 0]
    xe_history[:, t] = xe[:, 0]
    u_history[:, t] = u[:, 0]
    xe_last = xe
    P_last = P
    u_last = u

# save data
np.savetxt("x_history.out", x_history)

# plot trajectory
plt.figure(1).add_subplot(1, 3, 1, projection='3d')
plt.plot(x_history[0], x_history[1], x_history[2])
plt.axis('equal')
plt.title("x")
plt.figure(1).add_subplot(1, 3, 2, projection='3d')
plt.plot(y_history[0], y_history[1], y_history[2])
plt.axis('equal')
plt.title("y")
plt.figure(1).add_subplot(1, 3, 3, projection='3d')
plt.plot(xe_history[0], xe_history[1], xe_history[2])
plt.axis('equal')
plt.title("x̂")
plt.show()
