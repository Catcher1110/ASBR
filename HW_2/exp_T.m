function res = exp_T(S, theta)
% Calculate the exponential transformation matrix
w = S(1:3);
w_hat = skew_sym(w);
v = S(4:6);
res = [eye(3)+sin(theta)*w_hat+(1-cos(theta))*w_hat^2, (eye(3)*theta+(1-cos(theta))*w_hat+(theta-sin(theta))*w_hat^2)*v;
    zeros(1,3), 1];
end