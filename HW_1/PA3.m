%**************************************************************************
%Purpose: Program: calculate the transform matrix by using skew axis
% author:     Yang Liu     EID:yl34825
%             Jian Chu     EID:jc86537
%**************************************************************************
%% PA1_3
clear;
clc;
close all;
% define variables
q = [0,2,0];
s_hat = [0,0,1];
h = 2;
theta = pi;
T = [1 0 0 2;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
% Get the transform matrix 
T_tr = TransformM(q, s_hat, h, theta);
T1 = T_tr*T;
disp("The final configuration is T1:");
disp(T1);

T_tr = TransformM(q, s_hat, h, pi/4);
T2 = T_tr*T;
disp("The intermediate configuration at pi/4 is:");
disp(T2);

T_tr = TransformM(q, s_hat, h, pi/2);
T3 = T_tr*T;
disp("The intermediate configuration at pi/2 is:");
disp(T3);

T_tr = TransformM(q, s_hat, h, 3*pi/4);
T4 = T_tr*T;
disp("The intermediate configuration at 3pi/4 is:");
disp(T4);

quiver3(0,0,0,1,0,0);
hold on
quiver3(0,0,0,0,1,0);
quiver3(0,0,0,0,0,1);
% Plot final axes
P  = T1(1:3,4);
R  = T1(1:3,1:3);
x1 = R(1:3,1);
y1 = R(1:3,2);
z1 = R(1:3,3);
quiver3(P(1),P(2),P(3),x1(1),x1(2),x1(3));
quiver3(P(1),P(2),P(3),y1(1),y1(2),y1(3));
quiver3(P(1),P(2),P(3),z1(1),z1(2),z1(3));
% Plot intermediate axes at pi/4
P  = T2(1:3,4);
R  = T2(1:3,1:3);
x2 = R(1:3,1);
y2 = R(1:3,2);
z2 = R(1:3,3);
quiver3(P(1),P(2),P(3),x2(1),x2(2),x2(3));
quiver3(P(1),P(2),P(3),y2(1),y2(2),y2(3));
quiver3(P(1),P(2),P(3),z2(1),z2(2),z2(3));
% Plot intermediate axes at pi/2
P  = T3(1:3,4);
R  = T3(1:3,1:3);
x3 = R(1:3,1);
y3 = R(1:3,2);
z3 = R(1:3,3);
quiver3(P(1),P(2),P(3),x3(1),x3(2),x3(3));
quiver3(P(1),P(2),P(3),y3(1),y3(2),y3(3));
quiver3(P(1),P(2),P(3),z3(1),z3(2),z3(3));
% Plot intermediate axes at 3pi/4
P  = T4(1:3,4);
R  = T4(1:3,1:3);
x4 = R(1:3,1);
y4 = R(1:3,2);
z4 = R(1:3,3);
quiver3(P(1),P(2),P(3),x4(1),x4(2),x4(3));
quiver3(P(1),P(2),P(3),y4(1),y4(2),y4(3));
quiver3(P(1),P(2),P(3),z4(1),z4(2),z4(3));
% Plot intermediate axes at initial configuration
P  = T(1:3,4);
R  = T(1:3,1:3);
x = R(1:3,1);
y = R(1:3,2);
z = R(1:3,3);
quiver3(P(1),P(2),P(3),x(1),x(2),x(3));
quiver3(P(1),P(2),P(3),y(1),y(2),y(3));
quiver3(P(1),P(2),P(3),z(1),z(2),z(3));
axis equal
xlabel('x')
ylabel('y')
zlabel('z')

%% PA3_2
T1 =[-1 0 0 -2;
     0 -1 0 4;
     0  0 1 6.2832;
     0  0 0 1];
T0 =[1 0 0 0
     0 1 0 0
     0 0 1 0
     0 0 0 0];
exp_S1_theta1 = T0*T1^-1;
R1 = exp_S1_theta1(1:3,1:3);
[w, theta1] = skewlog(R1);
w_hat = [0, -w(3), w(2);
         w(3), 0, -w(1);
         -w(2), w(1), 0];
p1 = exp_S1_theta1(1:3,4);
v1 = skewv(theta1,p1',w_hat);
S1 = [w;v1];
disp("Take the rigid body from T1 to the origin: ");
disp("The screw axis S1 is:");
disp(S1);
disp("The distance Theta1 is:");
disp(theta1);

%%
% test correctness, use the same method in PA3_1, we can test whether 
% PA3_2 is correct. If it can go to the origin with the correct 
% orientation, it is correct. 
w         =S1(1:3);
v         =S1(4:6);
theta     =theta1;
w_hat = [0, -w(3), w(2);
         w(3), 0, -w(1);
         -w(2), w(1), 0];
R     = eye(3) + sin(theta)*w_hat + (1 - cos(theta))*w_hat^2;
star  = (eye(3)*theta + (1-cos(theta))*w_hat + (theta-sin(theta))*w_hat^2)*v;
T_tr  = [R star; 0 0 0 1];
T1 =[-1 0 0 -2;
     0 -1 0 4;
     0 0 1 6.2832;
     0 0 0 1];
T2 = T_tr*T1;
disp("Using the screw axis and distance is PA3_2, final configuration is:");
disp(T2);