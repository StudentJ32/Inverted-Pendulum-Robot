clc
clear

g = 9.8; %gravitational acc
Mr = 0.02245; %kg ,mass of the wheel
Mp = 0.22956; % kg mass of the body
R = 0.045085; % m radius of the wheel
L = 0.03424085; % distance between the z axis and the center of gravity of vehicle;
D = 0.24384; % m lateral distance between the contact patches of the wheel


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
Theta_d = 5*(pi/180); % degree disturbance angle
V_in = 0.00; % m/s input velocity
Psi_dot_in = 0*(pi/180); % (rad/s) input turn angular velocity


%gain design
% p1 = [-3-1.5j -3+1.5j -3-2j -3+2j];
% K1 = place(A_t,B_t,p1);
% 
p2 = [-0.5-3j -0.5+3j];
K2 = place(A_r,B_r,p2);
% 
%lqr
Q = zeros(4,4);
Q(4,4) = 1000;
Q(1,1) = 5000;
Q(3,3) = 10;
R = 1000;
K1 = lqr(A_t,B_t,Q,R);

sim t_wit_sim

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