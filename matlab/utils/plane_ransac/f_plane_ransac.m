function [model,inlinerIdx,varargout] = f_plane_ransac(pts_in,max_dis)
  sampleSize = 5;
  maxDistance = max_dis;
  fitFcn = @(pts) f_PlaneFitIn3D(pts);
  distFcn = @(model,pts) f_Plane3dVal(model,pts);
  [n_d,inlinerIdx] = ransac(pts_in',fitFcn,distFcn,sampleSize,maxDistance);
  pts_inler = pts_in(:,inlinerIdx);
  bar_p = mean(pts_inler,2);
  S = (pts_inler-bar_p)*(pts_inler-bar_p)';
  [U,Lambda] = eig(S);
  [U,Lambda]=sortDescend(double(U),double(Lambda));
  n =U(:,3);
  if (n'*bar_p)>0
    n = -n;
  end
  model = [n;bar_p];
  if nargout==3
      varargout{1} = f_calcPlaneCovariance(U,Lambda,pts_inler,0.02,0.05);
  end
end

function [U_o,Lambda_o]=sortDescend(U,Lambda)
lambdas = [Lambda(1,1),Lambda(2,2),Lambda(3,3)];
[B,I]=sort(lambdas,2,"descend");
U_o = U(:,I);
Lambda_o = eye(3);
Lambda_o(1,1) = B(1);
Lambda_o(2,2) = B(2);
Lambda_o(3,3) = B(3);

end