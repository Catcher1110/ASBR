%% This file is used to test our functions
%% Test for our PA 1
R = [1 0 0; 0 0 -1; 0 1 0];
[AA, Q, ZYZ, RPY] = TransformRO3(R);
disp(AA);
% The output is [1.0000, 0, 0, 1.5708]
% It means the axis is [1, 0, 0], and the angle is 1.5708
disp(Q);
% The output is [0.7071, 0.7071, 0, 0]
% It means 0.7071 + 0.7071i + 0j + 0k
disp(ZYZ);
% The output is [-1.5708; 1.5708; 1.5708] 
% This three values are the three angles
disp(RPY);
% The output is [-1.5708; 0; 0] 
% This three values are the three angles
%% Test for our PA 2
quat = [sqrt(2)/2, sqrt(2)/2, 0, 0];
w = [1, 0, 0];
theta = pi/2;
matR = Transform2RO3(quat);
disp(matR);
% The output is [1 0 0; 0 0 -1; 0 1 0], the same as the above section
matR = Transform2RO3(w, theta);
disp(matR);
% THe output is [1 0 0; 0 0 -1; 0 1 0], the same as the above section
%% Test for our PA 3, or you can just go to PA3.m to test by section
PA3