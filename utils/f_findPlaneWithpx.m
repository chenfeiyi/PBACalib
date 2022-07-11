function [p_model_o,p_cov]= f_findPlaneWithpx(pxs,pts_l_i,Tl2c,K)
p_model_o=[0,0,0,0];
[idx,dist] = f_findPtsfromPx(pxs,pts_l_i,Tl2c,K);
pts1 = pts_l_i(:,idx);
idx_valid = find(dist<5);
pts2 = pts1(:,idx_valid);
[p_model,inliers] = f_plane_ransac(pts2,0.06);
dists = p_model(1:3,1)'*(pts_l_i-p_model(4:6,1));
dists = abs(dists);
idxs = find(dists<0.06);
pts3 = pts_l_i(:,idxs);
[idx_range,d]=rangesearch(pts3',pts2',0.2);
idxs = [];
for i = 1:size(idx_range,1)
    idxs = [idxs,idx_range{i}];
end
idxs=unique(idxs);
pts4 = pts3(:,idxs);
% change this "maxdis" in f_plane_ransac function when the noise level
% changes in point cloud.
[p_model_o,inliers,p_cov] = f_plane_ransac(pts4,0.01); 
                                            
end