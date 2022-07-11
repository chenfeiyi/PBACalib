function [cam_pose_o]=f_optm_MLE_cam(detect_info,images,K,is_display)
% [1]"The Levenberg-Marquardt algorithm for nonlinear least squares
% curve-fitting problems"
if is_display
    disp("-------------f_optm_MLE_cam-------------");
end
remap=zeros(1,1000);
max_id = 0;
x0=[];
img_keys = keys(images);
for idx=1:size(images,1)
    id = img_keys{idx};
    lee_i = f_rotm2lee(images(id).R);
    x0 = [x0,double([lee_i';images(id).t])];
    remap(images(id).image_id) = idx;
    if images(id).image_id>max_id
        max_id= images(id).image_id;
    end
end
remap = remap(1:max_id);

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
last_iter=0;
while  iter<iter_max && func_step>func_step_min
    [f,H,g] = f_MLECam_obj(x,remap,K,detect_info);
    deltax = inv(H+lambda*diag(diag(H)))*g;
    x_new = x;
    deltax2 = reshape(deltax,[6,size(x,2)]);
    x_new(4:6,:) = x(4:6,:) + deltax2(4:6,:);

    for idx=1:size(x,2)
       x_new(1:3,idx) = f_rotm2lee(f_lee2rotm(deltax2(1:3,idx)')*f_lee2rotm(x(1:3,idx)));
    end
    f2 = f_MLECam_obj(x_new,remap,K,detect_info);
    rho = (f- f2)/(deltax'*(lambda*diag(diag(H))*deltax+g));
    if rho> e4
        x = x_new;
        lambda = max(lambda/2,1e-7);
        x_step = norm(deltax);
        if iter==0
            func_step = 10;
        else
            func_step = abs(last_loss - f);
        end
        last_iter = iter;
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

cam_pose_o = {};
for idx = 1:size(images,1)
    id = img_keys{idx};
    idx2 = remap(id);
    Ri = f_lee2rotm(x(1:3,idx2)');
    ti = x(4:6,idx2);
    cam_pose_o{id} = [[Ri,ti];[0,0,0,1]];
end
end

function [f,H,g]=  f_MLECam_obj(camTvec,remap,K,detect_info)
f=0;
H = zeros(6*size(camTvec,2));
g = zeros([6*size(camTvec,2),1]);
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
        err_ijk = f_obj_ijk(Ri,ti,Ri,ti,Rj,tj,s,K,n_li,bar_p_li,q_ik,u_jk);
        f = f+err_ijk'*err_ijk;
    end
end
if nargout>1
    for idx =1:size(detect_info,2)
        p_model = double(detect_info{idx}.p_model);
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
            err_ijk = f_obj_ijk(Ri,ti,Ri,ti,Rj,tj,s,K,n_li,bar_p_li,q_ik,u_jk);
            J_Ri_k = f_e_jocb_Ri_cam(Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_ti_k = f_e_jocb_ti_cam(Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_Rj_k = f_e_jocb_Rj(Ri,ti,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_tj_k = f_e_jocb_tj(Ri,ti,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_uik = f_e_jocb_uik(Ri,ti,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_ujk = f_e_jocb_ujk();
            J_wk = [J_uik;J_ujk]';
            J_xk = zeros(size(camTvec,2)*6,2)';
            J_xk(:,remap(img_idi)*6-5:remap(img_idi)*6) = [J_Ri_k;J_ti_k]';
            J_xk(:,remap(img_idj)*6-5:remap(img_idj)*6) = [J_Rj_k;J_tj_k]';
            sigma = px_cov;
            sigma_x = J_wk*sigma*J_wk';
            inv_sigma_x = inv(sigma_x);
            H = H+J_xk'*inv_sigma_x*J_xk;
            g = g-J_xk'*(inv_sigma_x)*err_ijk;
%             H = H+J_xk'*J_xk;
%             g = g-J_xk'*err_ijk;
        end
    end
end
end

function J_Ri_cam = f_e_jocb_Ri_cam(Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li)
d_Pik_Ri = f_derivative_Pk_R(Ri,ti,q_ik,n_li,bar_p_li);
p_ik = f_obj_pk(Ri,ti,q_ik,n_li,bar_p_li);
d_Pjk_Ri = (-f_skew(p_ik)*Ri+d_Pik_Ri*Ri)*Rj'+s*f_skew(ti)*Ri*Rj';
d_e_Pjk = f_derivative_e_Pjk(Ri,ti,Ri,Rj,ti,tj,s,K,q_ik,n_li,bar_p_li);
J_Ri_cam = d_Pjk_Ri*d_e_Pjk;
end
function J_ti_cam = f_e_jocb_ti_cam(Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li)
d_Pik_ti = f_derivative_Pk_t(Ri,q_ik,n_li);
d_Pjk_ti = d_Pik_ti*Ri*Rj'-s*Ri*Rj';
d_e_Pjk = f_derivative_e_Pjk(Ri,ti,Ri,Rj,ti,tj,s,K,q_ik,n_li,bar_p_li);
J_ti_cam = d_Pjk_ti*d_e_Pjk;
end