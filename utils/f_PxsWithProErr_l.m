function diff_o = f_PxsWithProErr_l(ui,uj, plane_cor,Tij, Tptoc1,K)
% ui: pixel feature points from camera 1
% uj: pixel feature points from camera 2
% plane_cor: the corresponding plane in camera1 frame
% Tptoc1: transformation from plane to camera 1
% Tij: transformation from camere 1 to camera 2
% K: camera intrinsic matrix
% diff_o: the porjection error.

pts3D = cross_pxs_plane(ui,plane_cor,Tptoc1,K);
pts3D_aft = Tij(1:3,1:3)*pts3D+Tij(1:3,4);
pts3D_aft_px = K*pts3D_aft;
pts3D_aft_px = pts3D_aft_px(1:2,:)./pts3D_aft_px(3,:);
diff = vecnorm(pts3D_aft_px - uj);
diff_o = diff;

end

function pk = cross_px_plane(uk,plane_model,T,K)
uk = [uk;1];
qk = inv(K)*uk;
pk = (T(1:3,4)'*T(1:3,1:3)*plane_model(1:3) ...
        +plane_model(4:6)'*plane_model(1:3))...
        /(qk'*T(1:3,1:3)*plane_model(1:3)) * qk;
end

function pts_cross = cross_pxs_plane(pxs,plane_model,T,K)
pts_cross=[];
for idx=1:size(pxs,2)
    pt = cross_px_plane(pxs(:,idx),plane_model,T,K);
    pts_cross = [pts_cross,pt];
end

end

function model_out = transform_plane(model_in,T)
model_out = model_in;
model_out(1:3,:) = T(1:3,1:3)*model_in(1:3,:);
model_out(4:6,:) = T(1:3,1:3)*model_in(4:6,:) + T(1:3,4);
end