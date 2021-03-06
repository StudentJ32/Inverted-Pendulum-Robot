    clc
clear

global J 

g = 9.8; %gravitational acc
Mr = 0.02245; %kg ,mass of the wheel
Mp = 2.5890; % kg mass of the body
R = 0.045085; % m radius of the wheel
L = 0.0306441; % distance between the z axis and the center of gravity of vehicle;
D = 0.25222; % m lateral distance between the contact patches of the wheel
J = 0.0188; % moment of inertia about rotational axis

R_m = 2.0313;
L_m= 0.0070389;
K_m= 1.082;
B = 0.0004729;

Den = [J*L_m (J*R_m+L_m*B) (B*R_m+K_m^2)];

%transptation system
A_t = zeros(4,4);
B_t = zeros(4,1);
X = (Mp*(Mp+6*Mr)*L)/(3*(Mp+1.5*Mr));
Y = Mp/((Mp+1.5*Mr)*R)+1/L;
A_t(1,2) = 1;
A_t(2,3) = g*(1-4*L*Mp/(3*X));
A_t(4,3) = g*Mp/X;
A_t(3,4) = 1;
B_t(2) = (4*L*Y/(3*X)-1/(Mp*L));
B_t(4) = -Y/X;
C_t = eye(4);
D_t = zeros(4,1);

%rotational system
A_r = [0 1;0 0];
B_r = [0;6/((9*Mr+Mp)*R*D)];
C_r = eye(2);
D_r = zeros(2,1);


% input control
Theta_d = 30*(pi/180); % degree disturbance angle
V_in = 0.00; % m/s input velocity
Psi_dot_in = 0*(pi/180); % (rad/s) input turn angular velocity

%gain design
p1 = [-3-2j -3+2j -7-6j -7+6j];
p2 = [-0.5-3j -0.5+3j];
K1 = place(A_t,B_t,p1);
K2 = place(A_r,B_r,p2);

%lqr
% Q = zeros(4,4);
% Q(4,4) = 10;
% Q(1,1) = 50;
% Q(3,3) = 5000;
% R = 1;
% K1 = lqr(A_t,B_t,Q,R);

K_p = 1.65;
K_i = 0;
K_d = 0;

sim t_wit_sim_w_motor_model

%torque
CL = (u1+u2)/2; % N*m left wheel torque
CR = u1-CL;% N*m right wheel torque

%plot
figure(1)
plot(t,[Xr Theta_p])
legend('position(m)','pitch angle(rad)')
figure(2)
plot(t,[Psi Psi_dot])
legend('yaw angle(rad)','yaw angle rate(rad/s)')
figure(3)
plot(t,[CL CR])
legend('left wheel torque(N*m)','right wheel torque(N*m)')
figure(4)
plot(t,[vol_L vol_R])
legend('left motor voltage supply','right motor voltage supply')

%% continious to discrete
Ts = 0.01;
sys_c = ss(A_t,B_t,C_t,D_t);
sys_d = c2d(sys_c,Ts,'zoh');
At_d = sys_d.a;
Bt_d = sys_d.b;
Ct_d = sys_d.c;
Dt_d = sys_d.d;
p1_d = exp(Ts*p1);
K1_d = place(At_d,Bt_d,p1_d)

sys_cr = ss(A_r,B_r,C_r,D_r);
sys_dr = c2d(sys_cr,Ts,'zoh');
Ar_d = sys_dr.a;
Br_d = sys_dr.b;
Cr_d = sys_dr.c;
Dr_d = sys_dr.d;
p2_d = exp(Ts*p2);
K2_d = place(Ar_d,Br_d,p2_d)