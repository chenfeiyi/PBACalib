function coeff = f_PlaneFitIn3D(pts)

% abc = -pinv(pts)*ones(size(pts,1),1);
abc = pts\ (-ones(size(pts,1),1));
coeff = [abc;1];
norm_abc = norm(abc);
coeff = coeff/norm_abc;
end
