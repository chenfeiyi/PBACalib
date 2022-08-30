function [TOptm_o,cam_pose_o,T_cov,xs]=f_optm_MLE_with_cam(detect_info,images,TInit,K,scale,is_display)
% [1]"The Levenberg-Marquardt algorithm for nonlinear least squares
% curve-fitting problems"
if is_display
    disp("------------f_optm_MLE_with_cam------------");
end
remap=zeros(1,1000);
max_id = 0;
x0=[];
img_keys = keys(images);

for idx=1:size(images,1)
    id = img_keys{idx};
    lee_i = f_rotm2lee(images(id).R);
    x0 = [x0,double([lee_i';images(id).t*scale])];
    remap(images(id).image_id) = idx;
    if images(id).image_id>max_id
        max_id= images(id).image_id;
    end
end
remap = remap(1:max_id);
x0 = [x0,[f_rotm2lee(TInit(1:3,1:3))';TInit(1:3,4)]];

x = x0;
iter =0;
x_step = 10;
last_loss=0;
func_step = 10;
lambda=10;
e1 = 400;
e2 = 1e-10;
e3 = 1e-6;
e4=0.5;
iter_max = 100;
x_step_min = 1e-6;
func_step_min = 1e-3;
Lambda_TT=[];
last_iter= 0;
xs = [];
while  iter<iter_max && func_step>func_step_min
    [f,H,g] = f_MLECam_obj(x(:,1:end-1),x(:,end),remap,K,detect_info);
    deltax = inv(H+lambda*diag(diag(H)))*g;
    x_new = x;
    deltax2 = reshape(deltax,[6,size(x,2)]);
    x_new(4:6,:) = x(4:6,:) + deltax2(4:6,:);

    for idx=1:size(x,2)
       x_new(1:3,idx) = f_rotm2lee(f_lee2rotm(deltax2(1:3,idx)')*f_lee2rotm(x(1:3,idx)));
    end
    f2 = f_MLECam_obj(x_new(:,1:end-1),x_new(:,end),remap,K,detect_info);
    rho = (f- f2)/(deltax'*(lambda*diag(diag(H))*deltax+g));
    if rho> e4
        x = x_new;
        xs = [xs,x_new(1:6,end)];
        lambda = max(lambda/2,1e-7);
        Lambda_TT = H(end-5:end,end-5:end);
        Lambda_CT = H(1:end-6,end-5:end);
        Lambda_TC = H(end-5:end,1:end-6);
        Lambda_CC = H(1:end-6,1:end-6);
        x_step = norm(deltax);
        if iter==0
            func_step = 10;
        else
            func_step = abs(last_loss - f);
        end
        last_iter=iter;
    else
        lambda = min(lambda*3,1e7);
        x_step = 0;
    end
    if max(abs(g))<e1 | max(abs(deltax2./x),[],'all')<e2 | func_step<e3 | (iter-last_iter)>5
        break;
    end
    last_loss = f;
    iter = iter + 1;
    if is_display
        disp("iter: "+num2str(iter)+" x_step: "+num2str(x_step)+" cost: "+num2str(f));
    end
end
if size(Lambda_TT,2)==0
    T_cov = 0;
else
    bar_Lambda_TT = Lambda_TT-Lambda_TC*pinv(Lambda_CC)*Lambda_CT;
    T_cov = inv(bar_Lambda_TT);
end
TOptm_o = [[f_lee2rotm(x(1:3,end)'),x(4:6,end)];[0,0,0,1]];
cam_pose_o = {};

for idx = 1:size(images,1)
    id = img_keys{idx};
    Ri = f_lee2rotm(x(1:3,idx)');
    ti = x(4:6,idx);
    cam_pose_o{id} = [[Ri,ti];[0,0,0,1]];
end
end

function [f,H,g]=  f_MLECam_obj(camTvec,Tvec,remap,K,detect_info)
R = f_lee2rotm(Tvec(1:3)');
t = Tvec(4:6);
f=0;
H = zeros(6*size(camTvec,2)+6);
g = zeros([6*size(camTvec,2)+6,1]);
K_inv = inv(K);
s=1;
for idx =1:size(detect_info,2)
    p_model = double(detect_info{idx}.p_model);
    ui = detect_info{idx}.ui;
    uj = detect_info{idx}.uj;
    n_li = p_model(1:3);
    bar_p_li = p_model(4:6);
    img_idi = detect_info{idx}.idx_i;
    img_idj = detect_info{idx}.idx_j;
    leei = camTvec(1:3,remap(img_idi));
    Ri = f_lee2rotm(leei);
    ti = camTvec(4:6,remap(img_idi));
    leej = camTvec(1:3,remap(img_idj));
    Rj = f_lee2rotm(leej);
    tj = camTvec(4:6,remap(img_idj));
    for idx2 = 1:size(ui,2)
        u_ik = ui(:,idx2);
        u_jk = uj(:,idx2);
        q_ik = [u_ik;1];
        q_ik = K_inv*q_ik;
        err_ijk = f_obj_ijk(R,t,Ri,ti,Rj,tj,s,K,n_li,bar_p_li,q_ik,u_jk);
        f = f+err_ijk'*err_ijk;
    end
end
J_x = zeros(size(camTvec,2)*6+6,2)';
if nargout>1
    for idx =1:size(detect_info,2)
        p_model = double(detect_info{idx}.p_model);
        p_cov = double(detect_info{idx}.p_cov);
        px_cov = 1.5*eye(4,4);
        ui = detect_info{idx}.ui;
        uj = detect_info{idx}.uj;
        n_li = p_model(1:3);
        bar_p_li = p_model(4:6);
        img_idi = detect_info{idx}.idx_i;
        img_idj = detect_info{idx}.idx_j;
        leei = camTvec(1:3,remap(img_idi));
        Ri = f_lee2rotm(leei);
        ti = camTvec(4:6,remap(img_idi));
        leej = camTvec(1:3,remap(img_idj));
        Rj = f_lee2rotm(leej);
        tj = camTvec(4:6,remap(img_idj));
        for idx2 = 1:size(ui,2)
            u_ik = ui(:,idx2);
            u_jk = uj(:,idx2);
            q_ik = [u_ik;1];
            q_ik = K_inv*q_ik;
            err_ijk = f_obj_ijk(R,t,Ri,ti,Rj,tj,s,K,n_li,bar_p_li,q_ik,u_jk);
            J_t_ijk = f_e_jocb_t(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_R_ijk = f_e_jocb_R(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_Ri_k = f_e_jocb_Ri(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_Rj_k = f_e_jocb_Rj(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_ti_k = f_e_jocb_ti(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_tj_k = f_e_jocb_tj(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_uik = f_e_jocb_uik(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_ujk = f_e_jocb_ujk();
            J_nijk = f_e_jocb_n(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_barP_ijk = f_e_jocb_barP(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_wk = [J_uik;J_ujk;J_nijk;J_barP_ijk]';
            J_xk = zeros(size(camTvec,2)*6+6,2)';
            J_xk(:,remap(img_idi)*6-5:remap(img_idi)*6) = [J_Ri_k;J_ti_k]';
            J_xk(:,remap(img_idj)*6-5:remap(img_idj)*6) = [J_Rj_k;J_tj_k]';
            J_xk(:,end-5:end) = [J_R_ijk;J_t_ijk]';
            sigma = blkdiag(px_cov,p_cov);
            sigma_x = J_wk*sigma*J_wk';
            inv_sigma_x = inv(sigma_x);
            H = H+J_xk'*inv_sigma_x*J_xk;
            g = g-J_xk'*(inv_sigma_x)*err_ijk;
            J_x = J_x+J_xk;
%             H = H+J_xk'*J_xk;
%             g = g-J_xk'*err_ijk; 
        end
    end
   
end
end
