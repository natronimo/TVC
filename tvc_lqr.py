import numpy as np
from scipy.spatial.transform import Rotation
from scipy.integrate import solve_ivp
import control as ct
import matplotlib.pyplot as plt

# rocket parameters
m = 170              # initial mass (kg)
r = 0.2              # rocket radius (m)
L = 2                # rocket length (m)
r_T = 1              # thrust moment arm (m)
I_sp = 250           # specific impulse (s)
T_s_magLim = 300     # x y axes thrust limit (N)
T_z_magLim = 2500    # z axis thrust limit (N)
T_s_rateLim = 600    # x y axes thrust rate limit (N/s)
# principal moments of inertia (kg*m^2)
I_x = lambda m: (1/4)*m*r**2 + (1/12)*m*L**2
I_y = lambda m: (1/4)*m*r**2 + (1/12)*m*L**2
I_z = lambda m: (1/2)*m*r**2

# simulation parameters
g = 9.81             # gravitational acceleration (m/s^2)
Ts = 0.01            # time step (s)
N = 1000             # total steps
x = np.array([[0], [0], [10], [0], [0], [0], [0], [0], [0], [0], [0], [0], [m]])    # initial state vector
u = np.array([[0], [0], [m*g], [0]])    # initial input vector
ref = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])    # reference state vector

# system matrix continuous
A = np.array([[0, 0, 0, 1, 0, 0,  0, 0, 0, 0, 0, 0],
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
              [0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0]])
# input matrix continuous
B = lambda m: np.array([[          0,          0,   0,        0],
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
                        [          0,          0,   0, 1/I_z(m)]])
# system matrix discrete
Ad = np.array([[1, 0, 0, Ts,  0,  0,          0, Ts**2*g/2, 0,          0, Ts**3*g/6,  0],
               [0, 1, 0,  0, Ts,  0, -Ts**2*g/2,         0, 0, -Ts**3*g/6,         0,  0],
               [0, 0, 1,  0,  0, Ts,          0,         0, 0,          0,         0,  0],
               [0, 0, 0,  1,  0,  0,          0,      Ts*g, 0,          0, Ts**2*g/2,  0],
               [0, 0, 0,  0,  1,  0,      -Ts*g,         0, 0, -Ts**2*g/2,         0,  0],
               [0, 0, 0,  0,  0,  1,          0,         0, 0,          0,         0,  0],
               [0, 0, 0,  0,  0,  0,          1,         0, 0,         Ts,         0,  0],
               [0, 0, 0,  0,  0,  0,          0,         1, 0,          0,        Ts,  0],
               [0, 0, 0,  0,  0,  0,          0,         0, 1,          0,         0, Ts],
               [0, 0, 0,  0,  0,  0,          0,         0, 0,          1,         0,  0],
               [0, 0, 0,  0,  0,  0,          0,         0, 0,          0,         1,  0],
               [0, 0, 0,  0,  0,  0,          0,         0, 0,          0,         0,  1]])
# input matrix discrete    
Bd = lambda m: np.array([[Ts**2/(2*m) - Ts**4*g*r_T/(24*I_y(m)),                                     0,           0,                0],
                         [                                    0, Ts**2/(2*m) - Ts**4*g*r_T/(24*I_x(m)),           0,                0],
                         [                                    0,                                     0, Ts**2/(2*m),                0],
                         [        Ts/m - Ts**3*g*r_T/(6*I_y(m)),                                     0,           0,                0],
                         [                                    0,         Ts/m - Ts**3*g*r_T/(6*I_x(m)),           0,                0],
                         [                                    0,                                     0,        Ts/m,                0],
                         [                                    0,                  Ts**2*r_T/(2*I_x(m)),           0,                0],
                         [                -Ts**2*r_T/(2*I_y(m)),                                     0,           0,                0],
                         [                                    0,                                     0,           0, Ts**2/(2*I_z(m))],
                         [                                    0,                         Ts*r_T/I_x(m),           0,                0],
                         [                       -Ts*r_T/I_y(m),                                     0,           0,                0],
                         [                                    0,                                     0,           0,        Ts/I_z(m)]])
C = np.eye(12)    # output matrix
Q = np.eye(12)    # state weight matrix
R = 0.01*np.eye(4)    # input weight matrix
G = np.eye(12)    # process noise matrix
QN = np.eye(12)    # process noise covariance matrix
RN = 10*np.eye(12)    # measurement noise covariance matrix
Ke, P, E = ct.dlqe(Ad, G, C, QN, RN)    # LQE matrix

# initialize arrays
x_history = np.empty((13, N))
y_history = np.empty((12, N))
xe_history = np.empty((12, N))
u_history = np.empty((4, N))
xe_last = x[0:12]
u_last = u

def dxdt(t, y):

    # velocity (m/s)
    v_x = y[3]
    v_y = y[4]
    v_z = y[5]
    # euler angles (rad)
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
    rotm = Rotation.from_euler('zyx', [theta_z, theta_y, theta_x])    # rotation matrix
    T_inertial = np.matmul(rotm.as_matrix(), T_body)    # thrust vector inertial frame (N)
    T_mag = np.linalg.norm(T_body)    # thrust magnitude (N)

    # intertial frame forces (N)
    F_x = T_inertial[0, 0]
    F_y = T_inertial[1, 0]
    F_z = T_inertial[2, 0] - m*g
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
                     omega_x,
                     omega_y,
                     omega_z,
                     (M_x - (I_z(m) - I_y(m))*omega_y*omega_z)/I_x(m),
                     (M_y - (I_x(m) - I_z(m))*omega_x*omega_z)/I_y(m),
                     (M_z - (I_y(m) - I_x(m))*omega_x*omega_y)/I_z(m),
                     m_dot])

for t in range(N):

    # integrate state vector through time
    sol = solve_ivp(dxdt, [0, Ts], np.transpose(x)[0], t_eval=[Ts])
    x = sol.y    # state vector

    m = x[12, 0]    # mass (kg)
    v = 0.1*np.random.standard_normal((12, 1))    # measurement noise vector

    y = np.matmul(C, x[0:12]) + v    # output vector

    xe = np.matmul(Ad, xe_last) + np.matmul(Bd(m), u_last)    # state estimate vector predict step
    xe = xe + np.matmul(Ke, y - np.matmul(C, xe))    # state estimate vector update step

    Kr, S, E = ct.lqr(A, B(m), Q, R)    # LQR matrix
    u = np.matmul(Kr, ref - xe) + np.array([[0], [0], [m*g], [0]])    # input vector

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
