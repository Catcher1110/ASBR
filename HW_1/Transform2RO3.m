function matR = Transform2RO3(varargin)
% Calculate the equivalent rotation matrix by given axis-angle
% or quaternion representation
% matR = Transform2RO3(varargin)
% Input:
%   axis-angle(3*1 vector and 1*1 scalar, two input) 
%   or quaternion representation (1*4 vector, one input)
% Output:
%   matR - Rotation matrix, which in SO(3)
% Example:
%   vec = [1, 0, 0, 0] - quaternion representation
%   w = [1, 0, 0] - axis
%   theta = pi/2 - angle
%   matR = Transform2RO3(vec)
%   matR = Transform2RO3(w, theta)
% Written by Jian Chu(jc86537)
% 2/19/2020, for ME397 ASBR

if nargin == 2 % it may be axis-angle
    w = varargin{1};
    theta = varargin{2};
    if size(w,2) ~= 3 || ~isa(w, 'float') || norm(w) == 0
        error('Wrong axis input!');
    elseif ~isa(theta, 'float')
        error('Wrong angle input! It should be a scalar!');
    else
        w = w / norm(w);
        w_hat = [0, -w(1,3), w(1,2);
                w(1,3), 0, -w(1,1);
                -w(1,2), w(1,1), 0];
        matR = eye(3) + w_hat * sin(theta) + w_hat * w_hat * (1-cos(theta));
    end
    
elseif nargin == 1 % it may be quaternion
    vec = varargin{1};
    if size(vec,2) == 4
        Magnitude = vec(1)^2 + vec(2)^2 + vec(3)^2 + vec(4)^2;
        if Magnitude > 0
            q0 = vec(1)/Magnitude;
            q1 = vec(2)/Magnitude;
            q2 = vec(3)/Magnitude;
            q3 = vec(4)/Magnitude;
            matR = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3);
                2*(q0*q3+q1*q2), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
                2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), q0^2-q1^2-q2^2+q3^2];
        else
            error('Error Quaternion input!(||q|| can not be zero)');
        end
    else
        error('Error Quaternion input!(It should be 1*4 vector)');
    end
else % wrong input
    error('Please input data with correct format!');
end
end