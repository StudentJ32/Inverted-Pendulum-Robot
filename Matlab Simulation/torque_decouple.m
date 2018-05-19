function [ w_dot ] = torque_decouple( u )
%TORQUE_DECOUPLE Summary of this function goes here
%   Detailed explanation goes here
    global J
    torque_L = (u(1)+u(2))/2;
    torque_R = u(1)-torque_L;
    w_dot = [torque_L torque_R]/J;
end

