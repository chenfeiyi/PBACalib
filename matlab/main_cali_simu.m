clc;
clear;
addpath("colmap/");
addpath("utils/plane_ransac/");
addpath("solver/");
addpath("utils/");
addpath("visualize/");
addpath("LM_OptmAll_solver/jocbian/");
addpath("LM_OptmAll_solver/obj/");
addpath("LM_OptmAll_solver/");
%% load parameters
%639.99764,639.9976491530514,640.5,360.5
D = [0.0, 0.0, 0.0, 0.0, 0.0];
K = [639.9976491530514, 0.0, 640.5;...
    0.0, 639.9976491530514, 360.5;...
    0.0, 0.0, 1.0];
TGt = [0.1564   -0.9877    0.0000    0.2807
    -0.0775   -0.0123   -0.9969    0.2108
    0.9846    0.1560   -0.0785   -0.1296
    0         0         0    1.0000];
delatT = [[eul2rotm([2*pi/180,3*pi/180,0]),[0.1,0.05,0]'];[0,0,0,1]];
TInit = delatT*TGt;
%% load image and point cloud data
data_path = "/home/cfy/Documents/livoxBACali/data/simu/noise-0.01/";
pcd_folder = data_path+"pcd";
img_folder = data_path+"img";
disp("***************load images and pcds*************");
[img_list,imgs_raw] = f_load_data(img_folder,'img');
[pcd_list,pcds_raw] = f_load_data(pcd_folder,'pcd');

%% preprocess: SFM to find all 3D points in the same scale
disp("***************read sfm model *************");
sfm_model_path = data_path+"models";
[cameras,images_raw,points3D] = read_model(char(sfm_model_path));
points3D = f_planeChooseInCamera(points3D);
r_errs=[];
t_errs=[];
r_errs1=[];
t_errs1=[];
for pose_num=4:10
    sub_r_errs=[];
    sub_t_errs=[];
    sub_t_errs1=[];
    sub_r_errs1=[];
    iter=1;
    while iter<51
        %% randomly select pose_num poses
        disp("pose number: "+num2str(pose_num)+" iter: "+num2str(iter));
        images = containers.Map('KeyType', 'int64', 'ValueType', 'any');
        num=1;
        reidx = randperm(images_raw.Count);
        for idx=reidx(1:pose_num)
            sub_image = images_raw(idx);
            images(sub_image.image_id)= sub_image;
        end
        %% camera pose optimization
        % disp("***************Camera Pose Optimization*************");
        img_keys = keys(images);
        [~,pts3d] = f_getPxPoints3D(images,points3D,img_keys{1});
        [p_model,inliers] = f_plane_ransac(pts3d,0.05);
        detect_info = f_dataPreprocess2(images,points3D,K,p_model);
        [cam_pose_o]=f_optm_MLE_cam(detect_info,images,K,false);
        for idx=1:size(images,2)
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
        if conf<4e-5
            disp("input data is not good");
            continue;
        end
%         disp("The intial transformation from lidar to camera is:");
%         disp(T1);
        %% optimize transformation
        disp("***************optimize transformation *************");
        % data preprocess
        disp("preprocessing data...");
        detect_info = f_dataPreprocess(images,points3D,pcds_raw,scale,T1,K);
        disp("start optimization...");
%         [T2,scale_o]=f_optm_MLE_with_s(detect_info,T1,K,double(scale),false);
        [T3,~,T_cov]=f_optm_MLE_with_cam(detect_info,images,T1,K,double(scale),false);
        [angle_err1,t_err1] = f_err_analysis(TGt,T1);
%         [angle_err2,t_err2] = f_err_analysis(TGt,T2);
        [angle_err3,t_err3] = f_err_analysis(TGt,T3);j
        sub_r_errs = [sub_r_errs;angle_err3];
        sub_t_errs = [sub_t_errs;t_err3];
        sub_r_errs1 = [sub_r_errs1;angle_err1];
        sub_t_errs1 = [sub_t_errs1;t_err1];
        iter = iter + 1;
    end
    r_errs = [r_errs,sub_r_errs];
    t_errs = [t_errs,sub_t_errs];
    r_errs1 = [r_errs1,sub_r_errs1];
    t_errs1 = [t_errs1,sub_t_errs1];
end
