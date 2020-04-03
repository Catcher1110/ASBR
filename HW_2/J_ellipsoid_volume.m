function mu3=J_ellipsoid_volume(J)

A         =J*J';
mu3       =sqrt(det(A));

end