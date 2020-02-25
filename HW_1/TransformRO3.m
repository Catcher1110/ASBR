function [AA, Q, ZYZ, RPY] = TransformRO3(matR)
% Transform to equivalent axis-angle, quaternion, ZYZ and
% roll-pitch-yaw representation by given rotation matrix R
% [AA, Q, ZYZ, RPY] = TransformRO3(matR)
% Input:
%   matR - rotation matrix, in SO(3)
% Output:
%   AA - axis-angle representation (1*4, [w1,w2,w3,angle])
%   Q - Quaternion representation (1*4, [q0, q1i, q2j, q3k])
%   ZYZ - ZYZ representation (3*1)
%   RPY - roll-pitch-yaw representation (3*1)

% Written by Jian Chu(jc86537) Yang Liu(yl34825)
% 2/19/2020, for ME397 ASBR

% Test Input
if size(matR,1)~=3 || size(matR,2)~=3 || abs(det(matR)-1) > 0.01 || norm(matR*matR'-eye(3)) > 0.01
    error('Wrong Input!');
end
% Axis-angle representation
theta = acos(0.5*(trace(matR)-1));
if theta == 0 % arbitrary axis is ok, we pick [1, 0, 0] here
    AA = [1, 0, 0, theta];
elseif theta == pi % Rotation for 180 degrees
    S = 1/(2*(1+matR(3,3)))^(1/2);
    AA = [matR(1,3)*S, matR(2,3)*S, (1+matR(3,3))*S, theta];
else % No singularity happens
    w_hat = (matR - matR')/(2*sin(theta));
    AA = [w_hat(3,2), w_hat(1,3), w_hat(2,1), theta];
end

% Quaternion representation
trR = trace(matR);
if trR > 0
    S = sqrt(trR + 1) * 2;
    q0 = 0.25 * S;
    q1 = (matR(3,2) - matR(2,3))/S;
    q2 = (matR(1,3) - matR(3,1))/S;
    q3 = (matR(2,1) - matR(1,2))/S;
elseif matR(1,1) > matR(2,2) && matR(1,1) > matR(3,3)
    S = sqrt(1 + matR(1,1) - matR(2,2) - matR(3,3)) * 2;
    q0 = (matR(3,2) - matR(2,3))/S;
    q1 = 0.25 * S;
    q2 = (matR(1,2) + matR(2,1))/S;
    q3 = (matR(1,3) + matR(3,1))/S;
elseif matR(2,2) > matR(3,3) 
    S = sqrt(1 + matR(2,2) - matR(1,1) - matR(3,3)) * 2;
    q0 = (matR(1,3) - matR(3,1))/S;
    q1 = (matR(1,2) + matR(2,1))/S;
    q2 = 0.25 * S;
    q3 = (matR(2,3) + matR(3,2))/S;
else
    S = sqrt(1 + matR(3,3) - matR(1,1) - matR(2,2)) * 2;
    q0 = (matR(2,1) - matR(1,2))/S;
    q1 = (matR(1,3) + matR(3,1))/S;
    q2 = (matR(2,3) + matR(3,2))/S;
    q3 = 0.25 * S;
end
Q = [q0, q1, q2, q3];
% ZYZ representation
theta = atan2(sqrt(matR(1,3)^2 + matR(2,3)^2), matR(3,3));
if theta == 0
    phi = 0;
    psi = atan2(matR(2,1), matR(1,1));
elseif theta == pi
    phi = 0;
    psi = -atan2(matR(2,1), matR(1,1));
else
    phi = atan2(matR(2,3), matR(1,3));
    psi = atan2(matR(3,2), -matR(3,1));
end
ZYZ = [phi; theta; psi];
% roll-pitch-yaw representation
beta = atan2(-matR(3,1), sqrt(matR(1,1)^2 + matR(2,1)^2));
if beta == pi/2
  alpha = 0;
  gamma = atan2(matR(1,2), matR(2,2));
elseif beta == -pi/2
  alpha = 0;
  gamma = -atan2(matR(1,2), matR(2,2));
else
  alpha = atan2(matR(2,1)/cos(beta), matR(1,1)/cos(beta));
  gamma = atan2(matR(3,2)/cos(beta), matR(3,3)/cos(beta));
end
RPY = [gamma; beta; alpha];

end