function mu2 =J_condition(J)

A         =J*J';
e         =eig(A);
lambda_max=max(e);
lambda_min=min(e);
mu2       =lambda_max/lambda_min;

end