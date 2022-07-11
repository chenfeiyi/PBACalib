function points3D_o = f_planeChooseInCamera(points3D)
% multiple planes may exist in the restored 3D scene
% this function will choose a plane, which contains most points.
points3D_o = points3D;

pt_val = values(points3D);
pts = [];
for idx=1:size(pt_val,2)
    pts = [pts,pt_val{idx}.xyz];
end

[model,inlier] = f_plane_ransac(pts,0.05);
pts_keys = keys(points3D);
for idx=1:points3D.Count
    x = points3D(pts_keys{idx});
    x.isplane = inlier(idx);
    points3D_o(pts_keys{idx})=x;
end

end