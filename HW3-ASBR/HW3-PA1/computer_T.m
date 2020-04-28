function [T,R,p]=computer_T(ai,bi)

% compute ai_tilae 
a_bar          = mean(ai);
ai_tilde       = ai-a_bar;
% compute bi_tilae
b_bar          = mean(bi);
bi_tilde       = bi-b_bar;
[n,~]          = size(bi_tilde);
H              = zeros(3,3);
for i = 1:n
    ax = ai_tilde(i,1);
    ay = ai_tilde(i,2);
    az = ai_tilde(i,3);
    bx = bi_tilde(i,1);
    by = bi_tilde(i,2);
    bz = bi_tilde(i,3);
    Hi =[ax*bx, ax*by, ax*bz;
           ay*bx, ay*by, ay*bz;
           az*bx, az*by, az*bz];
    H  =H + Hi;
end
[U,~,V]    = svd(H);
R          =V*U';
disp(det(R))
p          =b_bar'-R*a_bar';

T          =[R,   p;
             0 0 0 1];
end