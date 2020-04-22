function [mu3_w, mu3_v]=J_ellipsoid_volume(J)
Jw = J(1:3, :);
Jv = J(4:6, :);

Aw = Jw * Jw';
Av = Jv * Jv';
mu3_w       =sqrt(det(Aw));
mu3_v       =sqrt(det(Av));
end