function res = AdT(T)
% Calculate the Adjacent matrix for Transformation matrix
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    res = [R, zeros(3,3); skew_sym(p) * R, R];
end