function [w_axis, theta]=skewlog(R)
%**************************************************************************
%Purpose: calculate the log by using rotation matrix
%input: 
%point: rotation matix
%ouput:
% w_hat and theta
% author:     Yang Liu     EID:yl34825
%**************************************************************************
if R==eye(3)
  theta = 0;
  w_hat = zeros(3,3); %undefined
else
  %**********************************************************************
  cos_theta = (trace(R)-1)/2;
  if cos_theta==-1
      theta=pi;
      w_hat=(1/(2*(1+R(3,3)))^(1/2))*[R(1,3);R(2,3);(1+R(3,3))];
  else
      theta = acos(cos_theta);
      w_hat = (R-R')/(2*sin(theta));
  end
  %**********************************************************************
end

   w_axis = [w_hat(3,2);w_hat(1,3);w_hat(2,1)];
end
