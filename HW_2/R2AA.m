function [w, theta] = R2AA(matR)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Axis-angle representation
theta = acos(0.5*(trace(matR)-1));
if theta == 0 % arbitrary axis is ok, we pick [1, 0, 0] here
    w = [1, 0, 0];
elseif theta == pi % Rotation for 180 degrees
    S = 1/(2*(1+matR(3,3)))^(1/2);
    w = [matR(1,3)*S, matR(2,3)*S, (1+matR(3,3))*S];
else % No singularity happens
    w_hat = (matR - matR')/(2*sin(theta));
    w = [w_hat(3,2), w_hat(1,3), w_hat(2,1)];
end
end

