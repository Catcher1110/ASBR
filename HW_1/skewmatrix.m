function res = skewmatrix(p)
    res = zeros(3,3);
    res(3,2) = p(1);
    res(2,3) = -p(1);
    res(1,3) = p(2);
    res(3,1) = -p(2);
    res(2,1) = p(3);
    res(1,2) = -p(3);
end