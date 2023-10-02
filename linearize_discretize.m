close all; clear variables; clc

syms x y z v_x v_y v_z theta_x theta_y theta_z omega_x omega_y omega_z m
syms T_x T_y T_z M_z
syms g r_T I_x I_y I_z Ts tau

T_body = [T_x; T_y; T_z];
rotm = [cos(theta_y)*cos(theta_z), sin(theta_x)*sin(theta_y)*cos(theta_z) - cos(theta_x)*sin(theta_z), cos(theta_x)*sin(theta_y)*cos(theta_z) + sin(theta_x)*sin(theta_z)
        cos(theta_y)*sin(theta_z), sin(theta_x)*sin(theta_y)*sin(theta_z) + cos(theta_x)*cos(theta_z), cos(theta_x)*sin(theta_y)*sin(theta_z) - sin(theta_x)*cos(theta_z)
        -sin(theta_y), sin(theta_x)*cos(theta_y), cos(theta_x)*cos(theta_y)];
T_inertial = rotm*T_body;

x = [x; y; z; v_x; v_y; v_z; theta_x; theta_y; theta_z; omega_x; omega_y; omega_z];
u = [T_x; T_y; T_z; M_z];
dxdt = [v_x
        v_y
        v_z
        T_inertial(1)/m
        T_inertial(2)/m
        (T_inertial(3) - m*g)/m
        omega_x
        omega_y
        omega_z
        (T_body(2)*r_T - (I_z - I_y)*omega_y*omega_z)/I_x
        (-T_body(1)*r_T - (I_x - I_z)*omega_x*omega_z)/I_y
        (M_z - (I_y - I_x)*omega_x*omega_y)/I_z];

A = jacobian(dxdt, x);
B = jacobian(dxdt, u);
A = subs(A, [theta_x, theta_y, theta_z, omega_x, omega_y, omega_z, T_x, T_y, T_z], [0, 0, 0, 0, 0, 0, 0, 0, m*g]);
B = subs(B, [theta_x, theta_y, theta_z, omega_x, omega_y, omega_z, T_x, T_y, T_z], [0, 0, 0, 0, 0, 0, 0, 0, m*g]);
Ad = expm(A*Ts)
Bd = int(expm(A*tau)*B, tau, 0, Ts)