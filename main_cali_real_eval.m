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

TInit = [0.0324   -0.9994    0.0130   -0.0152
    0.0215   -0.0123   -0.9997    0.0695
    0.9992    0.0327    0.0211   -0.0132
         0         0         0    1.0000];
TInit(1:3,1:3) = eul2rotm([0.03,0.02,0.1])*TInit(1:3,1:3);
%% load image and point cloud data
data_path = "/home/cfy/Documents/livoxBACali/data/real/scene2/";
pcd_folder = data_path+"pcd";
img_folder = data_path+"img_un";
disp("***************load images and pcds*************");
[img_list,imgs_raw] = f_load_data(img_folder,'img');
[pcd_list,pcds_raw] = f_load_data(pcd_folder,'pcd');

%% preprocess: SFM to find all 3D points in the same scale
disp("***************read sfm model *************");
sfm_model_path = data_path+"models";
[cameras,images_raw,points3D] = read_model(char(sfm_model_path));
points3D = f_planeChooseInCamera(points3D);
iter =1;
Ts = {};
Ts2 = {};
while iter<21
    %% randomly select pose_num poses
    disp("iter: "+num2str(iter));
    images = containers.Map('KeyType', 'int64', 'ValueType', 'any');
    reidx = randperm(images_raw.Count);
    for idx=reidx(1:12)
        sub_image = images_raw(idx);
        images(sub_image.image_id)= sub_image;
    end
    %% camera pose optimization
    disp("***************Camera Pose Optimization*************");
    img_keys = keys(images);
    [~,pts3d] = f_getPxPoints3D(images,points3D,img_keys{1});
    [p_model,inliers] = f_plane_ransac(pts3d,0.05);
    detect_info = f_dataPreprocess2(images,points3D,K,p_model);
%     if size(detect_info,2)<7
%         continue;
%     end
    [cam_pose_o]=f_optm_MLE_cam(detect_info,images,K,false);
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

    [T1,scale,conf]= f_initT_withPlane(images,points3D,pcds_raw,TInit,K);
    if conf<1e-5
        disp("input data is not good");
        continue;
    end
    disp("The intial transformation from lidar to camera is:");
    disp(T1);

    disp("***************optimize transformation *************");
    % data preprocess
    detect_info = f_dataPreprocess(images,points3D,pcds_raw,scale,T1,K);
%     [T2,scale_o]=f_optm_MLE_with_s(detect_info,T1,K,double(scale),true);
    T1 = [ 0.0213   -0.9998    0.0022    0.0129
    0.0190   -0.0018   -0.9998    0.1038
    0.9996    0.0214    0.0189    0.0161
         0         0         0    1.0000];
    [T3,~,~,xs]=f_optm_MLE_with_cam(detect_info,images,T1,K,double(scale),true);
    Ts{iter}=T3;
    Ts2{iter} = T1;
    iter = iter+1;
end

ours_eul=[];
ours_t=[];
ours_eul_i=[];
ours_t_i=[];
for idx =1:size(Ts,2)
    T = Ts{idx};
    Ti = Ts2{idx};
    eul = rotm2eul(T(1:3,1:3),"XYZ")*180/pi;
    euli = rotm2eul(Ti(1:3,1:3),"XYZ")*180/pi;
    ours_eul = [ours_eul;eul];
    ours_t = [ours_t;T(1:3,4)'];
    ours_eul_i = [ours_eul_i;euli];
    ours_t_i = [ours_t_i;Ti(1:3,4)'];
end
ours_eul = ours_eul-[40,-88,52];
ours_eul_i = ours_eul_i-[40,-88,52];
xticks = ["roll","pitch","yaw"];
figure;
position_O = 1:2:5;
boxplot(ours_eul,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
position_O = position_O+1;
boxplot(ours_eul_i,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
figure;
xticks = ["x","y","z"];
position_O = 1:2:5;
boxplot(ours_t,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
position_O = position_O+1;
boxplot(ours_t_i,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
% disp("***************Visualize*************");
pc_name = pcd_folder + "/" +pcd_list(4,:);
pc_raw = pcread(pc_name);
% intensity = pc_raw.Intensity';
pts_in={};
pc_raw2 = pcdownsample(pc_raw,"gridAverage",0.0005);
pc_array = pc_raw2.Location()';
for idx=1:8
    [model,inlier]=f_plane_ransac(pc_array,0.02);
    pts_in{idx} = pc_array(:,inlier);
    pc_array= pc_array(:,~inlier);  
end

% img_pro = f_pro_livox2cam_i(pc_raw2.Location()',imgs_raw{1},K,T1,pc_raw2.Intensity');
% % img_pro = f_pro_livox2cam(pts_in,imgs_raw{1},K,T1);
% figure;
% title("Initial projection");
% hold on;
% imshow(img_pro);
% 
% img_pro = f_pro_livox2cam_i(pc_raw2.Location()',imgs_raw{1},K,T2,pc_raw2.Intensity');
% % img_pro = f_pro_livox2cam(pts_in,imgs_raw{1},K,T2);
figure;
title("Optimal T");
hold on; 
img_pro = f_pro_livox2cam_i(pc_raw2.Location()',imgs_raw{4},K,T3,pc_raw2.Intensity');
% img_pro = f_pro_livox2cam(pts_in,imgs_raw{7},K,T3);
imshow(img_pro);

