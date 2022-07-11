function error = f_Plane3dVal(model_coeff,points)
% points: dim*npts
points = [points';ones(1,size(points,1))];
norm_abc = norm(model_coeff(1:3));
model_coeff = model_coeff/norm_abc;

diss = abs(model_coeff'*points);
error = diss';
end
