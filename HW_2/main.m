%% Run this first!
% Define some variables
clear;
clc;
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
theta = [theta1 theta2 theta3 theta4 theta5 theta6 theta7];
M = [1 0 0 0.0880;
     0 -1 0 0;
     0 0 -1 0.9260;
     0 0 0 1];
ws = [0 0 1; 0 1 0; 0 0 1; 0 -1 0; 0 0 1; 0 -1 0; 0 0 -1]';
qs = [0 0 0.3330; 0 0 0.3330; 0 0 0.6490; 0.0825 0 0.6490;
     0 0 1.0330; 0 0 1.0330; 0.0880 0 1.0330]';
[~, num_Joint] = size(ws);
vs = zeros(3, num_Joint);
S = zeros(6, num_Joint);
for i = 1:1:num_Joint
    % Calculate the v_i = - cross(w_i, q_i)
    vs(:, i) = -cross(ws(:, i), qs(:, i));
    % Get the Screw Axis
    S(:, i) = [ws(:, i); vs(:, i)];
end

disp("Space Screws are: ");
disp(S);

wb = [0 0 -1; 0 -1 0; 0 0 -1; 0 1 0; 0 0 -1; 0 1 0; 0 0 1]';
qb = [-0.0880 0 0.5930; -0.0880 0 0.5930; -0.0880 0 0.2770; -0.0055 0 0.2770;
     -0.0880 0 -0.1070; -0.0880 0 -0.1070; 0 0 -0.1070]';
[~, num_Joint] = size(wb);
vb = zeros(3, num_Joint);
B = zeros(6, num_Joint);
for i = 1:1:num_Joint
    % Calculate the v_i = - cross(w_i, q_i)
    vb(:, i) = -cross(wb(:, i), qb(:, i));
    % Get the Screw Axis
    B(:, i) = [wb(:, i); vb(:, i)];
end

disp("Body Screws are: ");
disp(B);

%% Show Robot
robot = loadrobot("frankaEmikaPanda");
Name = {'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',...
    'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2'};
JointAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0};
initialConfiguration = struct('JointName', Name, 'JointPosition', JointAngle);
show(robot, initialConfiguration);

%% PA (a),(b)
FKs = FK_space(S, M, theta, qs);
disp("Forward Kinematics (Space Frame): ");
disp(FKs);

%% PA (c)
FKb = FK_body(B, M, theta, qb);
disp("Forward Kinematics (Body Frame): ");
disp(FKb);

%% PA (d) (e)
Js = J_space(S, theta);
disp("Space Jacobian:");
disp(Js);

Jb = J_body(B, theta);
disp("Body Jacobian:");
disp(Jb);

%% PA (g)
theta0 = [1 1 1 1 1 1 1];
Js0 = double(subs(Js, theta, theta0));
ellipsoid_plot(Js0);
[J_iso1, J_iso2] = J_isotropy(Js0);
[J_con1, J_con2]= J_condition(Js0);
[J_vol1, J_vol2]= J_ellipsoid_volume(Js0);
%% PA (h)
%desired configuration b
thetad = [1 1 1 1 1 1 1];
Tsd = double(subs(FKs, theta, thetad));
% initial guess
theta0 = [0 0 0 0 0 0 0];
% input body jacobian Jb
[AllTheta, AllT, Allwb, Allvb, ~] = J_inverse_kinematics(FKb,Jb,Tsd,theta0);

% Display Output
[n, ~] = size(AllTheta);
for i = 1:1:n
    fprintf('The angles for each joint after %d iterations are: \n', i);
    disp(AllTheta(i, :));
    fprintf('The configuration after %d iterations is: \n', i);
    disp(AllT{i});
end
disp("Desired Configuration is");
disp(Tsd);

% In the final Iteration, The norm of wb and vb are 0.000011, 0.000028.
% The Final Output are:
% The angles for each joint after 7 iterations are: 
%     0.3529    0.7037    0.2162   -0.0109   -1.0973   -1.3403   -2.4752
% 
% The configuration after 7 iterations is: 
%     0.5424   -0.0117   -0.8401    0.3002
%     0.7684    0.4111    0.4904    0.1803
%     0.3396   -0.9115    0.2320    0.8165
%          0         0         0    1.0000
% 
% Desired Configuration is
%     0.5424   -0.0117   -0.8401    0.3002
%     0.7684    0.4111    0.4904    0.1803
%     0.3396   -0.9115    0.2320    0.8165
%          0         0         0    1.0000

%% PA (i)
%desired configuration b
thetad = [1 1 1 1 1 1 1];
Tsd = double(subs(FKs, theta, thetad));
% initial guess
theta0 = [0 0 0 0 0 0 0];
% input body jacobian Jb
[AllTheta, AllT, ~, ~, ~] = J_transpose_kinematics(FKb,Jb,Tsd,theta0);
% Display Output
[n, ~] = size(AllTheta);
for i = 1:1:n
    fprintf('The angles for each joint after %d iterations are: \n', i);
    disp(AllTheta(i, :));
    fprintf('The configuration after %d iterations is: \n', i);
    disp(AllT{i});
end
disp("Desired Configuration is");
disp(Tsd);
% In the final Iteration, The norm of wb and vb are 0.000361, 0.000914.
% The Final Output are:
% The configuration after 150 iterations is: 
%     0.5422   -0.0114   -0.8402    0.3001
%     0.7685    0.4111    0.4903    0.1806
%     0.3398   -0.9115    0.2317    0.8156
%          0         0         0    1.0000
% 
% Desired Configuration is
%     0.5424   -0.0117   -0.8401    0.3002
%     0.7684    0.4111    0.4904    0.1803
%     0.3396   -0.9115    0.2320    0.8165
%          0         0         0    1.0000
%% PA (j)
%desired configuration b
thetad = [1 1 1 1 1 1 1];
Tsd = double(subs(FKs, theta, thetad));
% initial guess
theta0 = [0 0 0 0 0 0 0];
% input body jacobian Jb
[AllTheta, AllT, ~, ~,~] = redundancy_resolution(FKb,Jb,Tsd,theta0);
% Display Output
[n, ~] = size(AllTheta);
for i = 1:1:n
fprintf('The angles for each joint after %d iterations are: \n', i);
disp(AllTheta(i, :));
fprintf('The configuration after %d iterations is: \n', i);
disp(AllT{i});
end
disp("Desired Configuration is");
disp(Tsd);

% In the final Iteration, The norm of wb and vb are 0.191761, 0.476566.
% The Final Output are:
% The configuration after 7 iterations is: 
%     0.5424   -0.0117   -0.8400    0.3002
%     0.7684    0.4111    0.4904    0.1803
%     0.3396   -0.9115    0.2320    0.8165
%          0         0         0    1.0000
% 
% Desired Configuration is
%     0.5424   -0.0117   -0.8401    0.3002
%     0.7684    0.4111    0.4904    0.1803
%     0.3396   -0.9115    0.2320    0.8165
%          0         0         0    1.0000
%% PA (k)
%arbitrary configuration a
Tsb          =double(subs(FKs, theta, [0 0 0 0 0 0 0]));
%desired configuration b
thetad       =[1 1 1 1 1 1 1];
Tsd          =double(subs(FKs, theta, thetad));
% initial guess
theta0       =[0 0 0 0 0 0 0];
% input body jacobian Jb
[AllTheta, AllT, ~, ~, ~] = DLS_inverse_kinematics(FKb,Jb,Tsd,theta0);
% Display Output
[n, ~] = size(AllTheta);
for i = 1:1:n
    fprintf('The angles for each joint after %d iterations are: \n', i);
    disp(AllTheta(i, :));
    fprintf('The configuration after %d iterations is: \n', i);
    disp(AllT{i});
end
disp("Desired Configuration is");
disp(Tsd)

% In the final Iteration, The norm of wb and vb are 0.000482, 0.000398.
% The Final Output are:
% The angles for each joint after 19 iterations are: 
%     1.2959    1.9596   -3.5342    5.2364    5.7417    1.1966    8.0185
% 
% The configuration after 19 iterations is: 
%    -0.5412    0.4842   -0.6875   -0.0643
%     0.2725   -0.6724   -0.6881    0.6618
%    -0.7955   -0.5598    0.2320    0.5176
%          0         0         0    1.0000
% 
% Desired Configuration is
%    -0.5412    0.4842   -0.6875   -0.0643
%     0.2725   -0.6724   -0.6881    0.6618
%    -0.7955   -0.5598    0.2320    0.5176
%          0         0         0    1.0000