%% calculate the ground truth of the transformation in Gazebo
xyz=[0.1 0.3 0.2]; % parameters in Gazebo
rpy=[0 pi/40 pi/20]; % parameters in Gazebo

R = eul2rotm([rpy(3),rpy(2),rpy(1)]);
t = xyz';
DeltaT = [0,0,1,0;
     -1,0,0,0;
      0,-1,0,0;
      0,0,0,1]';
T1 = [[R,t];[0,0,0,1]];
TGt = DeltaT*inv(T1);
% visualize
pcd_raw = pcread("/home/cfy/Documents/livoxBACali/data/simu/pcd/12399.163000.pcd");
pcd_raw = pcdownsample(pcd_raw,"gridAverage",0.1);
pc_array = pcd_raw.Location()';
img = imread("/home/cfy/Documents/livoxBACali/data/simu/img/12399.069000.jpg");
K = [959.9964737295771, 0.0, 960.5;...
    0.0, 959.9964737295771, 540.5;...
    0.0, 0.0, 1.0];
img_pro = pt_project_depth2image(TGt,K,pc_array,img);

imshow(img_pro);
