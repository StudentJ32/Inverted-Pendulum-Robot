function [ torques ] = torque_sum( u )
%TORQUE_SUM Summary of this function goes here
%   Detailed explanation goes here
    torques = [(u(1)+u(2)) (u(1)-u(2))];
end

