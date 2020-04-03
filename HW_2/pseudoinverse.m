function [Jstar]=pseudoinverse(Jb)
%calculate the dimensions of jacobian
% J is body jacobian
%[m, n] = size(Jb);
%calculate pseudoinverse
%if n==m
    Jstar=pinv(Jb);
% elseif n>m
%     Jstar=Jb'*inv(Jb*Jb'); 
% else
%     Jstar=inv(Jb'*Jb)*Jb'; 
% end

end