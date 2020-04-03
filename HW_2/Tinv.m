function res = Tinv(T)
%TINV Get the inverse of the T matrix
%   T - 4 * 4 matrix
R = T(1:3, 1:3);
P = T(1:3, 4);
res = [R', -R' * P; 0, 0, 0, 1];
end

