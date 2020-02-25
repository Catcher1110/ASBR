function T_tr = TransformM(q, s_hat, h, theta)
% Calculate the transform matrix
% T_tr = TransformM(q, s_hat, h, theta)
% Input:
%   point: q, unit direction: s_hat, pitch: h and angle: theta
% Output:
%   transform matrix
% Written by Jian Chu(jc86537) Yang Liu(yl34825)
% 2/19/2020, for ME397 ASBR

w = s_hat;
v = -cross(s_hat,q)+h*s_hat;
S = [w';v'];
w_hat = [0, -w(3), w(2);
         w(3), 0, -w(1);
         -w(2), w(1), 0];
% Rotation Matrix
R = eye(3) + sin(theta)*w_hat + (1 - cos(theta))*w_hat^2;
% Calculate the translation_vector
star=(eye(3)*theta + (1-cos(theta))*w_hat + (theta-sin(theta))*w_hat^2)*v';
% Form transform matrix T_tr
T_tr =[R star; 0 0 0 1];
end