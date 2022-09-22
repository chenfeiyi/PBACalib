function [T_o,scale_o,confidence,lambda_min] = f_initT_withPlane(images,points3D,pcds_raw,TInit,K)
% TInit: transformation from lidar to camera
% T_o: transformation from camera to lidar
%% Initialize Rotation first
pts_ref = [];
pts_mov =[];
T_o = eye(4);
scale_o = 1;
pts_val = values(points3D);
pts3d =[];
for idx=1:points3D.Count
    pts3d = [pts3d,pts_val{idx}.xyz];
end
[p_model_cam0,inliers]=f_plane_ransac(pts3d,0.04);
img_key = keys(images);
for idx=1:size(images,1)
    R = images(img_key{idx}).R;
    t = images(img_key{idx}).t;
    p_model = zeros(4,1);
    p_model(1:3) = R*p_model_cam0(1:3);
    p_model(4:6) = R*p_model_cam0(4:6)+t;
    pts_ref =[pts_ref,p_model(:,1)];
    
    [px,pts3d] = f_getPxPoints3D(images,points3D,img_key{idx});
    [idx2,dist] = f_findPtsfromPx(px,pcds_raw{img_key{idx}},TInit,K);
    pts_from_pcd = pcds_raw{img_key{idx}};
    sub_pcd1 = pts_from_pcd(:,idx2);
    sub_pcd1 = sub_pcd1(:,find(abs(dist)<10));
    [p_model,inliers]=f_plane_ransac(sub_pcd1,0.03);
    sub_pcd1 = sub_pcd1(:,inliers);
    if size(sub_pcd1,2)<5
        confidence=0;
        lambda_min=0;
        return;
    end
    dist  = p_model(1:3)'*(pts_from_pcd-p_model(4:6));
    inliers = find(abs(dist)<0.06);
    sub_pcd = pts_from_pcd(:,inliers);
    [p_model,inliers]=f_plane_ransac(sub_pcd,0.01);
    pts_mov=[pts_mov,p_model(:,1)];
end
T_o(1:3,1:3) = f_RotationSVDSolver(pts_ref(1:3,:),pts_mov(1:3,:));
R = T_o(1:3,1:3);
%% Initialize translation and scale
A=[];
b=[];
for idx=1:images.Count
    n_li = pts_mov(1:3,idx);
    bar_p_li = pts_mov(4:6,idx);
    n_ci = pts_ref(1:3,idx);
    bar_p_ci = pts_ref(4:6,idx);
    Ai = [n_ci'*bar_p_ci,-n_ci'];
    bi = n_ci'*R*bar_p_li;
    A = [A;Ai];
    b = [b;bi];
end
st = A\b;
scale_o = st(1);
t = st(2:4);
T_o(1:3,4) = t;
[V,D]= eig(A'*A);
confidence = D(1,1)/D(4,4);
lambda_min = D(1,1);
end



function R = f_RotationSVDSolver(pt_ref,pt_mov)
% pt_ref: dim*npts
% pt_mov: dim*npts

q_ref =pt_ref;
q_mov =pt_mov;

Dim = size(pt_mov,1);
H = zeros(3,3);
for idx = 1:size(q_ref,2)
    H = H + q_mov(:,idx)*q_ref(:,idx)';
end
[U,S,V] = svd(H);

X = V*U';

detX = det(X);
R = [];
if detX-1 > -0.0001
    R = X;
elseif detX<0
    B = eye(Dim);
    B(Dim,Dim) = det(V*U');
    R = V*B*U';
end

end