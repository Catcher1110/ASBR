function [AA, Q, ZYZ, RPY] = TransformRO3(matR)
% Calculate equivalent axis-angle, quaternion, ZYZ and
% roll-pitch-yaw representation by given rotation matrix R
% [AA, Q, ZYZ, RPY] = TransformRO3(matR)
% Input:
%   matR - rotation matrix, in SO(3)
% Output:
%   AA - axis-angle representation
%   Q - Quaternion representation

% Written by Jian Chu(jc86537)
% 2/19/2020, for ME397 ASBR

% Test Input
if size(matR,1)~=3 || size(matR,2)~=3 || det(matR)~=1 || norm(matR*matR'-eye(3))
    error('Wrong Input!');
end
% Axis-angle representation
theta = acos(0.5*(trace(matR)-1));
if theta == 0 % arbitrary axis is ok, we pick [1, 0, 0] here
    AA = [1, 0, 0, theta];
elseif theta == pi %
    xx = (matR(1,1)+1)/2;
    yy = (matR(2,2)+1)/2;
    zz = (matR(3,3)+1)/2;
    xy = (matR(1,2)+matR(2,1))/4;
    xz = (matR(1,3)+matR(3,1))/4;
    yz = (matR(2,3)+matR(3,2))/4;
    if matR(1,1)>matR(2,2) && matR(1,1)>matR(3,3)
        if matR(1,1) == -1
            AA = [0, sqrt(2)/2, sqrt(2)/2, theta]; 
        else
            x = sqrt(xx);
            AA = [x, xy/x, xz/x, theta];
        end
    elseif matR(2,2)>matR(3,3)
        if matR(2,2) == -1
            AA = [sqrt(2)/2, 0, sqrt(2)/2, theta];
        else
            y = sqrt(yy);
            AA = [xy/y, y, yz/y, theta];
        end
    else
        if matR(3,3) == -1
            AA = [sqrt(2)/2, sqrt(2)/2, 0, theta];
        else
            z = sqrt(zz);
            AA = [xz/z, yz/z, z, theta];
        end
    end
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

% roll-pitch-yaw representation

end