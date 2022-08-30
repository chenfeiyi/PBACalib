function [idx,dist] = f_findPtsfromPx(px,pts3d,T,K)

pts3d = T(1:3,1:3)*pts3d+T(1:3,4);
pts_aft_px = K*pts3d;
pts_aft_px = pts_aft_px(1:2,:)./pts_aft_px(3,:);

ns = createns(pts_aft_px');
[idx, dist] = knnsearch(ns,px','k',1);

end