%%%This main file for the experiment of success rate



clc;
clear;
addpath("colmap/");
addpath("utils/plane_ransac/");
addpath("utils/");
addpath("visualize/");
addpath("LM_solver/jocbian/");
addpath("LM_solver/obj/");
addpath("LM_solver/");
%% load parameters
% 897.4566,896.7992,635.4040,375.3149
K = [897.4566,0,635.4040;
    0,896.7992,375.3149;
    0,0,1];
D = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
 
Tgt=[0.0269   -0.9996    0.0101 -0.0018
    0.0217   -0.0095   -0.9997 0.0657
    0.9994    0.0271    0.0215 -0.0202
    0         0         0    1.0000];
%% uniformly sample points on the surface of unit sphere

[V,Tri]=SpiralSampleSphere(50,false);
setupR = [2,5,10,15,20,25,30];
setupt = [0.1,0.15,0.2,0.25,0.3,0.35,0.4]; 
% r_errs_s={};
% t_errs_s={};
%% load image and point cloud data
data_path = "/home/cfy/Documents/livoxBACali/data/sub_real1";
pcd_folder = data_path+"/pcd";
img_folder = data_path+"/img";

disp("***************load images and pcds*************");
[img_list,imgs_raw] = f_load_data(img_folder,'img');
[pcd_list,pcds_raw] = f_load_data(pcd_folder,'pcd');

%% preprocess: SFM to find all 3D points in the same scale
disp("***************read sfm model *************");
sfm_model_path = data_path+"/models";
[cameras,images,points3D] = read_model(char(sfm_model_path));
points3D = f_planeChooseInCamera(points3D);
for setupnum=1:7
    r_errs=[];
    t_errs=[];
    iter =1;
    theta_err = setupR(setupnum)*pi/180;
    t_err_norm = setupt(setupnum);
    while iter<=50
        
        %% calculate the new initial parameters.
        del_R = f_lee2rotm(theta_err*V(iter,:));
        del_t = t_err_norm*V(iter,:);
        TInit = Tgt;
        TInit(1:3,1:3) = del_R*TInit(1:3,1:3);
        TInit(1:3,4) = TInit(1:3,4)+del_t';
        q = rotm2quat(del_R);
        disp("iter: "+num2str(iter)+"  R err: "+num2str(2*acosd(q(1)))+" degree t error: "+num2str(norm(del_t))+"m");
        %% camera pose optimization
        disp("***************Camera Pose Optimization*************");
        img_keys = keys(images);
        [~,pts3d] = f_getPxPoints3D(images,points3D,img_keys{1});
        [p_model,inliers] = f_plane_ransac(pts3d,0.05);
        detect_info = f_dataPreprocess2(images,points3D,K,p_model);

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
            iter = iter + 1;
            r_errs=[r_errs,10];
            t_errs=[t_errs,10];
            continue;
        end
        disp("***************optimize transformation *************");
        % data preprocess
        detect_info = f_dataPreprocess(images,points3D,pcds_raw,scale,T1,K);
        [T3,~,~,xs]=f_optm_MLE_with_cam(detect_info,images,T1,K,double(scale),false);
        disp(T3);
        Terr = inv(T3)*Tgt;
        q_err = rotm2quat(Terr(1:3,1:3));
        r_err = 2*acosd(q_err(1));
        t_err = norm(T3(1:3,4) - Tgt(1:3,4));
        r_errs=[r_errs,r_err];
        t_errs=[t_errs,t_err];
        iter = iter+1;
        disp("Final result  R err: "+num2str(r_err)+" degree t error: "+num2str(norm(t_err))+"m");
    end
    r_errs_s{setupnum}=r_errs;
    t_errs_s{setupnum}=t_errs;
    
end


