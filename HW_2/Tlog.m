function [w, v, theta] = Tlog(T)
%TLOG Summary of this function goes here
%   Detailed explanation goes here
R = T(1:3, 1:3);
p = T(1:3, 4);
if norm(R - eye(3)) <= 1e-6
    w = [0; 0; 0];
    v = p / norm(p);
    theta = norm(p);
else
    % Axis-angle representation
    theta = acos(0.5*(trace(R)-1));
    if theta == 0 % arbitrary axis is ok, we pick [1, 0, 0] here
        w = [1; 0; 0];
    elseif theta == pi % Rotation for 180 degrees
        S = 1/(2*(1+matR(3,3)))^(1/2);
        w = [R(1,3)*S; R(2,3)*S; (1+R(3,3))*S];
    else % No singularity happens
        w_hat = (R - R')/(2*sin(theta));
        w = [w_hat(3,2); w_hat(1,3); w_hat(2,1)];
    end
    
    if theta == 0
        v = [0; 0; 0];
    else
        Ginv = eye(3) / theta - 0.5 * skew_sym(w) + (1/theta - 0.5 * cot(theta/2))*skew_sym(w)^2;
        v = Ginv * p;
    end
end
end


