%% HW Problem 1
syms L1 L2 L3 L4 L5 L6
w = [1 0 0; 0 0 -1; 0 1 0; 1 0 0; 0 0 0; 0 1 0];
q1 = [L1 0 0; L1 0 0; L1 L3 L2; L1 L3 0; L1 L3+L4 0; L1 L3+L4 -L5];
q2 = [-L1 -L3-L4 L5+L6; 0 -L3-L4 L5+L6; 0 -L3-L4 L2+L5+L6; 
      0 -L4 L5+L6; 0 0 L5+L6; 0 0 L6];
for i = 1:1:6
    disp(i);
    disp(-cross(w(i,:), q1(i,:)));
end

for j = 1:1:6
    disp(j);
    disp(-cross(w(j,:), q2(j,:)));
end
%% HW Problem 2
clc;
w = [0 0 1; 0 0 0; 0 0 1; 0 -1 0; -sqrt(2)/2 0 sqrt(2)/2];
q1 = [0 0 -1; 0 0 0; 1 0 0; 1 0 -1; 2 0 -1];
q2 = [-3 0 -1; -3 0 0; -2 0 0; -2 0 -1; -1 0 -1];
S = [];
B = [];
for i = 1:1:5
    Si = [w(i,:)'; -cross(w(i,:), q1(i,:))'];
    S = [S, Si];
end
disp("Space Jacobian");
disp(S);
for i = 1:1:5
    Bi = [w(i,:)'; -cross(w(i,:), q2(i,:))'];
    B = [B, Bi];
end
disp("Body Jacobian");
disp(B);
%% HW Problem 3
close all;
clear;
clc;
syms L theta1 theta2 theta3 theta4
% calculate Si
w1=[0; 0; 1];
q1=[0; 0; L];
v1=-cross(w1, q1);
s1=[w1;v1];

w2=[cos(theta1); sin(theta1); 0];
q2=[0; 0; L];
v2=-cross(w2, q2);
s2=[w2;v2];

w3=Rot_z(theta1)*Rot_x(theta2)*[0; 0; 1];
q3=[0; 0; L]+Rot_z(theta1)*Rot_x(theta2)*[0; L; 0];
v3=-cross(w3, q3);
s3=[w3;v3];

w4=[0; 0; 0];
v4=Rot_z(theta1)*Rot_x(theta2)*Rot_z(theta3)*[0; 1; 0];
s4=[w4;v4];

Js_theta=[s1 s2 s3 s4];
Js_theta0=subs(Js_theta, [theta1, theta2, theta3, theta4], [0,0,pi/2,L]);
theta=[1;1;1;1];
p_dot=Js_theta0*theta;
disp(p_dot);

%calculate transformation matrix Ti
R1=Rot_z(theta1);
p1=[0;0;0];
T1=[R1     p1;
    0 0 0   1];
R2=Rot_x(theta2);
p2=[0;0;L];
T2=[R2      p2;
    0 0 0   1];
R3=Rot_z(theta3);
p3=[0;L;0];
T3=[R3      p3;
    0 0 0   1];
R4=eye(3);
p4=[0;theta4;0];
T4=[R4      p4;
    0 0 0   1];
%calculate rotation matrix R(t) and displacement matrix P(t)
Tsb=T1*T2*T3*T4;
R=Tsb(1:3,1:3);
P=Tsb(1:3,4);
[P_hat]=skew_sym(P);
o=zeros(3,3);
%calculate adjoint matrix [AdTbs]
AdTbs=[R'         o;
       -R'*P_hat  R'];
Jb_theta=AdTbs*Js_theta;
Jb_theta0=subs(Jb_theta, [theta1, theta2, theta3, theta4], [0,0,pi/2,L]);
disp(Jb_theta0);

%% HW Problem 3-2
clc;
syms L theta1 theta2 theta3 theta4
% calculate Si
w1=[0; 0; 1];
q1=[0; 0; 0];
v1=-cross(w1, q1);
s1=[w1;v1];

w2= [1; 0; 0];
q2=[0; 0; L];
v2=-cross(w2, q2);
s2=[w2;v2];

w3=[0; 0; 1];
q3=[0; L; L];
v3=-cross(w3, q3);
s3=[w3;v3];

w4=[0; 0; 0];
v4=[0; 1; 0];
s4=[w4;v4];

Js = [s1, AdT(exp_T(s1, theta1))*s2, AdT(exp_T(s1, theta1)*exp_T(s2, theta2))*s3,...
      AdT(exp_T(s1, theta1)*exp_T(s2, theta2)*exp_T(s3, theta3))*s4];
Js0 = subs(Js, [theta1, theta2, theta3, theta4], [0,0,pi/2,L]);
disp(Js0);
thetadot = [1;1;1;1];
pdot = Js0 * thetadot;
disp(pdot);

q1b = [0;-L;-L];
v1b = -cross(w1, q1b);
b1 = [w1;v1b];

q2b = [0;-L;0];
v2b = -cross(w2, q2b);
b2 = [w2;v2b];

q3b = [0;0;0];
v3b = -cross(w3, q3b);
b3 = [w3;v3b];

v4b = [0; 1; 0];
b4 = [w4;v4b];

Jb = [AdT(exp_T(-b4, theta4)*exp_T(-b3, theta3)*exp_T(-b2, theta2))*b1,...
      AdT(exp_T(-b4, theta4)*exp_T(-b3, theta3))*b2,...
      AdT(exp_T(-b4, theta4))*b3, b4];
Jb0 = subs(Jb, [theta1, theta2, theta3, theta4], [0,0,pi/2,L]);
disp(Jb0);
%% HW Problem 4 Method 1
clc;
syms L theta1 theta2 theta3 theta4 theta5 theta6
% calculate Si
w1=[0; 0; 1];
q1=[0; 0; 0];
v1=-cross(w1, q1);
s1=[w1;v1];

w2=Rot_z(theta1) * [0;1;0];
q2=[0; 0; 0];
v2=-cross(w2, q2);
s2=[w2;v2];

w3=Rot_z(theta1)*Rot_y(theta2)*[-1; 0; 0];
q3=[0; 0; 0]+Rot_z(theta1)*Rot_x(theta2)*[0; 0; 0];
v3=-cross(w3, q3);
s3=[w3;v3];

w4=Rot_z(theta1)*Rot_y(theta2)*Rot_x(-theta3)*[-1; 0; 0];
q4=[0;0;0]+Rot_z(theta1)*Rot_y(theta2)*Rot_x(-theta3)*[0; L; 0];
v4=-cross(w4, q4);
s4=[w4;v4];

w5=Rot_z(theta1)*Rot_y(theta2)*Rot_x(-theta3)*Rot_x(-theta4)*[-1; 0; 0];
q5=q4+Rot_z(theta1)*Rot_y(theta2)*Rot_x(-theta3)*Rot_x(-theta4)*[0; L; 0];
v5=-cross(w5, q5);
s5=[w5;v5];

w6=Rot_z(theta1)*Rot_y(theta2)*Rot_x(-theta3)*Rot_x(-theta4)*Rot_x(-theta5)*[0; 1; 0];
q6=q5+Rot_z(theta1)*Rot_y(theta2)*Rot_x(-theta3)*Rot_x(-theta4)*Rot_x(-theta5)*[0; L; 0];
v6=-cross(w6, q6);
s6=[w6;v6];

Js_theta=[s1 s2 s3 s4 s5 s6];
disp(simplify(Js_theta));
detJ = det(Js_theta);
facJ = factor(detJ);
disp(simplify(facJ));

%% HW Problem 4 Method 2

syms L theta1 theta2 theta3 theta4 theta5 theta6
% calculate Si
w1=[0; 0; 1];
q1=[0; 0; 0];
v1=-cross(w1, q1);
s1=[w1;v1];

w2=[0;1;0];
q2=[0; 0; 0];
v2=-cross(w2, q2);
s2=[w2;v2];

w3=[-1; 0; 0];
q3=[0; 0; 0];
v3=-cross(w3, q3);
s3=[w3;v3];

w4=[-1; 0; 0];
q4=[0; L; 0];
v4=-cross(w4, q4);
s4=[w4;v4];


w5=[-1; 0; 0];
q5=[0; 2*L; 0];
v5=-cross(w5, q5);
s5=[w5;v5];

w6=[0; 1; 0];
q6=[0; 3*L; 0];
v6=-cross(w6, q6);
s6=[w6;v6];

Js = [s1, AdT(exp_T(s1, theta1))*s2, AdT(exp_T(s1, theta1)*exp_T(s2, theta2))*s3,...
      AdT(exp_T(s1, theta1)*exp_T(s2, theta2)*exp_T(s3, theta3))*s4,...
      AdT(exp_T(s1, theta1)*exp_T(s2, theta2)*exp_T(s3, theta3)*exp_T(s4, theta4))*s5,...
      AdT(exp_T(s1, theta1)*exp_T(s2, theta2)*exp_T(s3, theta3)*exp_T(s4, theta4)*exp_T(s5, theta5))*s6];
disp(simplify(Js));

detJ = det(Js);
facJ = factor(detJ);
disp(simplify(facJ));
