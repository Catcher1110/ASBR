function [mu2_w, mu2_v] = J_condition(J)
Jw = J(1:3, :);
Jv = J(4:6, :);

Aw = Jw * Jw';
Av = Jv * Jv';

ew         =eig(Aw);
lambda_max_w=max(ew);
lambda_min_w=min(ew);
mu2_w       =lambda_max_w/lambda_min_w;

ev         =eig(Av);
lambda_max_v=max(ev);
lambda_min_v=min(ev);
mu2_v       =lambda_max_v/lambda_min_v;
end