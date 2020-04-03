function Jb = J_body(S, Theta)
% Calculate the Body Jacobian
% Input:  S - Screw Info, each column is corresponding to the each joint
%         Theta - Input for each joint
% Output: Jb - Body form Jacobian

[~, num_Joint] = size(S);
Jb = [];
for i = 1:1:num_Joint-1
    exp_Term = 1;
    for j = num_Joint:-1:i+1
        exp_Term = exp_Term * exp_T(-S(:, j), Theta(j));
    end
    Jb = [Jb, AdT(exp_Term)*S(:, i)];
end
Jb = [Jb, S(:, num_Joint)];
end