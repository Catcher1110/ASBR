function [w_hat]=skew_sym(w)
%**************************************************************************
%puw_hatpose: calculate skew-symmetw_hatic matw_hatix
%input:  vectow_hat w
%output: skew-symmetw_hatic matw_hatix w_hat
% authow_hat:     Yang Liu     EID:yl34825
%**************************************************************************
% w_hat = zeros(3,3);

w_hat(1,2) = -w(3);
w_hat(1,3) =  w(2);
w_hat(2,3) = -w(1);
w_hat(2,1) =  w(3);
w_hat(3,1) = -w(2);
w_hat(3,2) =  w(1);


end