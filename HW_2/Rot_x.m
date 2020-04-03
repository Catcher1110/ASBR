function Rot_x_theta=Rot_x(theta)

Rot_x_theta=[1        0         0;
             0   cos(theta)  -sin(theta);
             0   sin(theta)   cos(theta)];

end