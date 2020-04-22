function [AllTheta, AllT, Allwb, Allvb, AllJb0] = J_transpose_kinematics(FKb,Jb,Tsd,theta0)
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7
theta = [theta1 theta2 theta3 theta4 theta5 theta6 theta7];
% set epsilonw epsilonv
epsilonw        =0.01;
epsilonv        =1e-3;
wb              =[1;1;1];
vb              =[1;1;1];
i = 0;
AllTheta = theta0;
AllT = {double(subs(FKb, theta, theta0))};
Allwb = norm(wb);
Allvb = norm(vb);
AllJb0 = {double(subs(Jb, theta, theta0))};
dt = 0.1;
while (norm(wb)>epsilonw) || (norm(vb)>epsilonv)
    %calculate Tbs
    Tbs_thetai              =Tinv(double(subs(FKb, theta, theta0)));
    %calculate Tbd
    Tbd_thetai              =Tbs_thetai*Tsd;
    %calculate twist Vb
    R                       =Tbd_thetai(1:3,1:3);
    p                       =Tbd_thetai(1:3,4);
    [w_hat, Theta]          =skewlog(R);
    [v]                     =skewv(Theta,p,w_hat);
    wb                      =[w_hat(3,2);w_hat(1,3);w_hat(2,1)] * Theta;
    vb                      =v * Theta;
    Vb                      =[wb;vb];
    %-------------------------------------------------------------
    %update theta i
    Jb0            =double(subs(Jb, theta, theta0));
    alpha          =dot(Vb,Jb0*Jb0'*Vb)/dot(Jb0*Jb0'*Vb,Jb0*Jb0'*Vb);
    theta_iplus1   =theta0+(alpha*Jb0'*Vb)' * dt;
    theta0         =theta_iplus1;
    
    i = i + 1;
    
    normwb = norm(wb);
    normvb = norm(vb);
    Allwb = [Allwb, normwb];
    Allvb = [Allvb, normvb];
    fprintf('Iteration %d: The norm of wb and vb are %.6f, %.6f. \n', i, normwb, normvb);

    Tactual = double(subs(FKb, theta, theta0));
    AllJb0{end+1} = double(subs(Jb, theta, theta0));
    AllTheta = [AllTheta; theta0];
    AllT{end+1} = Tactual;
%     fprintf('The angles for each joint after %d iterations are: \n', i);
%     disp(theta0);
%     fprintf('The configuration after %d iterations is: \n', i);
%     disp(Tactual);
end