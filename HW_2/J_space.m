function Js = J_space(S, Theta)
% Calculate the Space Jacobian
% Input:  S - Screw Info, each column is corresponding to the each joint
%         Theta - Input for each joint
% Output: Js - Space form Jacobian

[~, num_Joint] = size(S);
Js = S(:,1);
for i = 2:1:num_Joint
    exp_Term = 1;
    for j = 1:1:i-1
        exp_Term = exp_Term * exp_T(S(:, j), Theta(j));
    end
    Js = [Js, AdT(exp_Term)*S(:, i)];
end
end