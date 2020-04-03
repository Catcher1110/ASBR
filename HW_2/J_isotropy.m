function mu1=J_isotropy(J)

A         =J*J';
e         =eig(A);
lambda_max=max(e);
lambda_min=min(e);
mu1       =sqrt(lambda_max/lambda_min);

end