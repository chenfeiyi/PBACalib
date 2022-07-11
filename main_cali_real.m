clc;
clear;
addpath("colmap/");
addpath("utils/plane_ransac/");
addpath("utils/");
addpath("visualize/");
addpath("LM_OptmAll_solver/jocbian/");
addpath("LM_OptmAll_solver/obj/");
addpath("LM_OptmAll_solver/");
%% load parameters
% 897.4566,896.7992,635.4040,375.3149
K = [897.4566,0,635.4040;
    0,896.7992,375.3149;
    0,0,1];
D = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];

TInit = [0.0324   -0.9994    0.0130   0.0
    0.0215   -0.0123   -0.9997    0.0
    0.9992    0.0327    0.0211   0
    0         0         0    1.0000];
%% load image and point cloud data
data_path = "/home/cfy/Documents/livoxBACali/data/real/scene3/";
pcd_folder = fullfile(data_path,"pcd");
img_folder = fullfile(data_path,"img_un");
disp("***************load images and pcds*************");
[img_list,imgs_raw] = f_load_data(img_folder,'img');
[pcd_list,pcds_raw] = f_load_data(pcd_folder,'pcd');

%% preprocess: SFM to find all 3D points in the same scale
disp("***************read sfm model *************");
sfm_model_path = data_path+"models";
[cameras,images,points3D] = read_model(char(sfm_model_path));
points3D = f_planeChooseInCamera(points3D);
%% camera pose optimization
disp("***************Camera Pose Optimization*************");
img_keys = keys(images);
[~,pts3d] = f_getPxPoints3D(images,points3D,img_keys{1});
[p_model,inliers] = f_plane_ransac(pts3d,0.05);
detect_info = f_dataPreprocess2(images,points3D,K,p_model);

[cam_pose_o]=f_optm_MLE_cam(detect_info,images,K,true);
for idx=1:size(images,1)
    id = img_keys{idx};
    sub_image = images(id);
    T = cam_pose_o{id};
    sub_image.R = T(1:3,1:3);
    sub_image.t = T(1:3,4);
    images(id) = sub_image;
end
%% initialize transformation
disp("***************Initialize transformation *************");

[T1,scale,conf,lambda_min]= f_initT_withPlane(images,points3D,pcds_raw,TInit,K);
if conf<4e-4 && lambda_min<4e-3 
    disp("The distribution of the input data is not good");
    return;
end
disp("The intial transformation from lidar to camera is:");
disp(T1);
disp("***************optimize transformation *************");
% data preprocess
disp("data preprocessing...");
detect_info = f_dataPreprocess(images,points3D,pcds_raw,scale,T1,K);
%     [T2,scale_o]=f_optm_MLE_with_s(detect_info,T1,K,double(scale),true);
disp("start calibration");
[T3,~]=f_optm_MLE_with_cam(detect_info,images,T1,K,double(scale),true);
disp("The final transformation from lidar to camera is:");
disp(T3);
disp("***************Visualize*************");
pc_name = pcd_folder + "/" +pcd_list(1,:);
pc_raw = pcread(pc_name);
pts_in={};
pc_raw2 = pcdownsample(pc_raw,"gridAverage",0.05);
figure;
title("Optimal T");
hold on;
% project points using intensity information
img_pro = f_pro_livox2cam_i(pc_raw.Location()',imgs_raw{1},K,T3,pc_raw.Intensity');
imshow(img_pro);
% project points using depth information
% img_pro = f_pt_project_depth2image(T3,K,pc_raw2.Location()',imgs_raw{1});
% imshow(img_pro);

