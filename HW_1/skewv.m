function [v]=skewv(theta,p,w_hat)
%**************************************************************************
%Purpose: calculate v by using theta,p,w_hat
%input: 
%theta,p,w_hat
%ouput:
%v
% author:     Yang Liu     EID:yl34825
%**************************************************************************
G_1theta     =1/theta*eye(3)-1/2*w_hat+(1/theta-1/2*cot(theta/2))*w_hat^2;
v            =G_1theta*p';

end