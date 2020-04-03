function res = FK_space(S, M, theta, q)
% Forward Space POE: T = e^[S_1]theta_1 * e^[S_2]theta_2 * ... * M
% Input:  q -
%         S - Space Screws
%         M - End effector configuration
%         theta - Input for each joint
% Output:

[~, num_Joint] = size(S);
res = 1;
for i = 1:1:num_Joint
    res = res * exp_T(S(:, i), theta(i));
end
res = res * M;

if nargin < 4
    return
else
    % Draw the Screw Axis and frames
    figure(1);
    title("Space Forward Kinematic Plot(Frames and Screw Axes)");
    hold on;
    grid on
    quiver3(0, 0, 0,0, 0, 1, 0.6, 'LineStyle', '-', 'Color', 'b');
    for i = 1:1:num_Joint
        w = S(1:3, i);
        v = S(4:6, i);
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