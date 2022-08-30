function detect_info_o = f_dataPreprocess2(images,points3D,K,p_model)
detect_info = {};
detect_info_o={};
num=1;
proerr_all=[];
proerrs={};
img_keys = keys(images);
for idx=1:images.Count-1
    for idx2 = idx+1:images.Count
        id = img_keys{idx};
        id2 = img_keys{idx2};
        [ui,uj,pts3d]=f_getPairedPxPt3D(images,points3D,id,id2);
        Ti = [[images(id).R,images(id).t];[0,0,0,1]];
        Tj = [[images(id2).R,images(id2).t];[0,0,0,1]];
        if size(ui,2)<5
            continue;
        end
        proerr  = f_PxsWithProErr_c(Ti,Tj,K,ui,uj,p_model);
        proerrs{num} = proerr;
        proerr_all =[proerr_all,proerr];
        detect_info{num}.idx_i = id;
        detect_info{num}.idx_j = id2;
        detect_info{num}.ui = ui;
        detect_info{num}.uj= uj;
        detect_info{num}.p_model = p_model;
        detect_info{num}.Ti= Ti;
        detect_info{num}.Tj= Tj;
        num = num + 1;
    end
end

% filter outliers and take 95% percent of main pts will be considered
proerrs_aft = sort(proerr_all);
thres_idx = int32(0.95*size(proerr_all,2));
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