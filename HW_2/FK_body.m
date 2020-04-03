function res = FK_body(B, M, Theta, q)
% Forward Body POE: T = M * e^[B_1]theta_1 * e^[B_2]theta_2 * ...
% Input:  B - Body Screws
%         M - End effector configuration
%         theta - Input for each joint
% Output:

[~, num_Joint] = size(B);
res = M;
for i = 1:1:num_Joint
    res = res * exp_T(B(:, i), Theta(i));
end

if nargin < 4
    return
else
    % Draw the Screw Axis and frames
    figure(1);
    title("Body Forward Kinematic Plot(Frames and Screw Axes)");
    hold on;
    for i = 1:1:num_Joint
        w = B(1:3, i);
        v = B(4:6, i);
        position = q(:, i);
        if w(1) ~= 0 || w(2) ~= 0 || w(3) ~= 0
            % It is a revolute joint
            quiver3(position(1), position(2), position(3), w(1), w(2), w(3),...
                    0.6, 'LineStyle', '-', 'Color', 'r');
        else
            % It is a prismatic joint
            quiver3(position(1), position(2), position(3), v(1), v(2), v(3),...
                    0.6, 'LineStyle', '-', 'Color', 'b');
        end
    end
end
end