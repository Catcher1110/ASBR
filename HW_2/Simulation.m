% This file only can run in Matlab 2020a right now.

% Load Robot Model(Valid only for Matlab 2020a)
robot = loadrobot("frankaEmikaPanda");
Name = {'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',...
    'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2'};
JointAngle = {0, 0, 0, 0, 0, 0, 0, 0, 0};
initialConfiguration = struct('JointName', Name, 'JointPosition', JointAngle);
show(robot, initialConfiguration);
pause(0.1);

%% Defined Robot Parameters
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

% Get Forward Kinematics
FKs = FK_space(S, M, theta);
FKb = FK_body(B, M, theta);

% Get Jacobian
Js = J_space(S, theta);
Jb = J_body(B, theta);

%% desired configuration b
thetad = [1 2 1 1 0 1 1];
Tsd = double(subs(FKs, theta, thetad));
% initial guess
theta0 = [0 0 0 0 0 0 0];
% input body jacobian Jb
[AllTheta, AllT] = J_transpose_kinematics(FKb,Jb,Tsd,theta0);

% Display Output
[n, ~] = size(AllTheta);
Name = {'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',...
    'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2'};
for i = 1:1:n
%     fprintf('The angles for each joint after %d iterations are: \n', i);
%     disp(AllTheta(i, :));
    JointAngle = {AllTheta(i,1), AllTheta(i,2), AllTheta(i,3), AllTheta(i,4),...
                    AllTheta(i,5), AllTheta(i,6), AllTheta(i,7), 0, 0};
%     fprintf('The configuration after %d iterations is: \n', i);
%     disp(AllT{i});
    configuration_i = struct('JointName', Name, 'JointPosition', JointAngle);
    show(robot, configuration_i);
    pause(0.1);
end
% disp("Desired Configuration is");
% disp(Tsd);