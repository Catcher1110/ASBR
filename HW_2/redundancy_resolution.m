function [AllTheta, AllT, Allwb, Allvb, AllJb0] = redundancy_resolution(FKb,Jb,Tsd,theta0)
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
theta = [theta1 theta2 theta3 theta4 theta5 theta6 theta7];
% set epsilonw epsilonv
epsilonw        =0.01;
epsilonv        =1e-3;
wb              =[1;1;1];
vb              =[1;1;1];
% k0>0, but how can we get k0 value?
k0              =1;
q0_dot          =zeros(1,7);
i = 0;
AllTheta = theta0;
AllT = {double(subs(FKb, theta, theta0))};
Allwb = norm(wb);
Allvb = norm(vb);
dt = 0.1;
AllJb0 = {double(subs(Jb, theta, theta0))};
while (norm(wb)>epsilonw) || (norm(vb)>epsilonv)
    %calculate Tbs
    Tsb_thetai              =double(subs(FKb, theta, theta0));
    Tbs_thetai              =Tinv(Tsb_thetai);
    %calculate Tbd
    Tbd_thetai              =Tbs_thetai*Tsd;
    %calculate twist Vb
    R                       =Tbd_thetai(1:3,1:3);
    p                       =Tbd_thetai(1:3,4);
    [w_hat, Theta]          =skewlog(R);
    [v]                     =skewv(Theta,p,w_hat);
    wb                      =[w_hat(3,2);w_hat(1,3);w_hat(2,1)] * Theta*dt;
    vb                      =v * Theta*dt;
    Vb                      =[wb;vb];
    %-------------------------------------------------------------
    %update theta i
    %---------------------------------------------------------------
    
    
    
    
    %---------------------------------------------------------------
    Jb0            =double(subs(Jb, theta, theta0));
    Wq0            =sqrt(det(Jb0*Jb0'));
    [Jstar0]       =pseudoinverse(Jb0);
    %since the homogeneous solution is always zero
    theta_iplus1   =theta0+(Jstar0*Vb)'+((eye(7)-Jstar0*Jb0)*q0_dot')';
    % this part is for geting q0_dot
    [q0_dot]=partial_differential(Jb,theta,theta0,theta_iplus1);
%     Jb1            =double(subs(Jb, theta, theta_iplus1));
%     Wq1            =sqrt(det(Jb1*Jb1'));
%     delta_q        =theta_iplus1-theta0;
%     delta_w        =Wq1-Wq0;
%     q0_dot         =delta_w*(zeros(1,7)+1)./delta_q;
    
    theta0         =theta_iplus1;
    i = i + 1;
    
    normwb = norm(wb);
    normvb = norm(vb);
    Allwb = [Allwb, normwb];
    Allvb = [Allvb, normvb];
    fprintf('Iteration %d: The norm of wb and vb are %.6f, %.6f. \n', i, normwb, normvb);
    % joint space limits
%     qLimit = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
%               -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
%     
    % Output Log
    Tactual = double(subs(FKb, theta, theta0));   
%     fprintf('The angles for each joint after %d iterations are: \n', i);
%     disp(theta0);
%     fprintf('The configuration after %d iterations is: \n', i);
%     disp(Tactual);
    AllJb0{end+1} = double(subs(Jb, theta, theta0));
    AllTheta = [AllTheta; theta0];
    AllT{end+1} = Tactual;
end