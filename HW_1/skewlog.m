function [w_hataxis, theta]=skewlog(R)
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
  w_hataxis = zeros(3,3); %undefined
else
  %**********************************************************************
  cos_theta = (trace(R)-1)/2;
  if cos_theta==-1
      theta=pi;
      w_hataxis=(1/(2*(1+R(3,3)))^(1/2))*[R(1,3);R(2,3);(1+R(3,3))];
  else
      theta = acos(cos_theta);
      w_hataxis = (R-R')/(2*sin(theta));
  end
  %**********************************************************************
end
end
