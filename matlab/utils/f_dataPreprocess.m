function detect_info_o = f_dataPreprocess(images,points3D,pcds_raw,scale,Tl2c,K)
detect_info = {};
detect_info_o=[];
num=1;
proerrs = [];
proerrs_all = [];
img_keys = keys(images);
for idx=1:images.Count-1
    id = img_keys{idx};
    pc_pts = pcds_raw{id};
    %find pixel on the plane in idx-th image 
    [pxs1_o,pxs2_o,pts_o]=f_getPairedPxPt3D(images,points3D,id,id);
    [plane_cor,p_cov]= f_findPlaneWithpx(pxs1_o,pc_pts,Tl2c,K);
    if isempty(plane_cor)
        detect_info_o=[];
        return;
    end
    % the calculation of plane covariance is so time consuming
    for idx2=idx+1:images.Count
        id2 = img_keys{idx2};
        [pxs_i,pxs_j,pts3d] = f_getPairedPxPt3D(images,points3D,id,id2);
        if size(pxs_i,2)<10
            continue;
        end
        T_ci = [[images(id).R,images(id).t];[0,0,0,1]];
        T_cj = [[images(id2).R,images(id2).t];[0,0,0,1]];
        T_ij = T_cj*inv(T_ci);
        T_ij(1:3,4) = scale*T_ij(1:3,4);
        sub_proerrs = f_PxsWithProErr_l(pxs_i,pxs_j,plane_cor,T_ij,Tl2c,K);
        proerrs_all = [proerrs_all,sub_proerrs];
        proerrs{num} = sub_proerrs;
        detect_info{num}.idx_i = id;
        detect_info{num}.idx_j = id2;
        detect_info{num}.ui = pxs_i;
        detect_info{num}.uj= pxs_j;
        detect_info{num}.p_model = plane_cor;
        detect_info{num}.p_cov = p_cov;
        detect_info{num}.T_i= T_ci;
        detect_info{num}.T_j= T_cj;
        detect_info{num}.scale= scale;
        num = num+1;
    end
end

% filter outliers and take 95% percent of main pts will be considered
proerrs_aft = sort(proerrs_all);
thres_idx = int32(0.95*size(proerrs_all,2));
thres = proerrs_aft(thres_idx);
pro_ub = 10;
pro_lb = 4;
num =1;
for idx = 1:size(detect_info,2)
    sub_proerrs = proerrs{idx};
    inliers = find((sub_proerrs<thres|sub_proerrs<pro_lb)&sub_proerrs<pro_ub);
    if size(inliers,2)<5
        continue;
    end
    detect_info{idx}.ui = detect_info{idx}.ui(:,inliers);
    detect_info{idx}.uj = detect_info{idx}.uj(:,inliers);
    detect_info_o{num} = detect_info{idx};
    num=num+1;
end

end