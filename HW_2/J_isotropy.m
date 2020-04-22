function [mu1_w, mu1_v]=J_isotropy(J)
Jw = J(1:3, :);
Jv = J(4:6, :);

Aw = Jw * Jw';
Av = Jv * Jv';

ew         =eig(Aw);
lambda_max_w=max(ew);
lambda_min_w=min(ew);
mu1_w       =sqrt(lambda_max_w/lambda_min_w);

ev         =eig(Av);
lambda_max_v=max(ev);
lambda_min_v=min(ev);
mu1_v       =sqrt(lambda_max_v/lambda_min_v);
end