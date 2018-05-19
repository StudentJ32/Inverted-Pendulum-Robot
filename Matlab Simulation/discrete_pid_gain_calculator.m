clc
clear all

Kp = 1;
Ki = 200;
Kd = 0;
N = 100;
Ts = 0.01;

a0 = (1+N*Ts);    
a1 = -(2 + N*Ts);
a2 = 1;
b0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
b1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
b2 = Kp + Kd*N;
ku1 = a1/a0; ku2 = a2/a0; ke0 = b0/a0; ke1 = b1/a0; ke2 = b2/a0;

