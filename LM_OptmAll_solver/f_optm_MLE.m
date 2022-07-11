% clc;
% clear;
% load("../detect.mat");
% addpath("jocbian/");
% addpath("obj/");
% s = double(detect_info{1}.scale);
% [TOptm_o,s_optm]=f_optm_with_s2(detect_info,T1,K,s)
function [TOptm_o]=f_optm_MLE(detect_info,TInit,K,scale)
iter_max = 100;
x_step_min = 1e-6;
func_step_min = 1e-3;
R = TInit(1:3,1:3);
t = TInit(1:3,4);
x0 = [f_rotm2lee(R),t'];
x = x0;
iter =0;
x_step = 10;
last_loss=0;
func_step = 10;
lambda=10;
e1 = 400;
e2 = 1e-10;
e3 = 1e-3;
e4=0.5;
while  iter<iter_max && func_step>func_step_min
    [f,H,g] = f_MLE_obj(x(1:3),x(4:6),scale,K,detect_info);
    deltax = inv(H+lambda*diag(diag(H)))*g;
    x_new = x;
    x_new(4:6) = x(4:6) + deltax(4:6)';
    x_new(1:3) = f_rotm2lee(f_lee2rotm(deltax(1:3)')*f_lee2rotm(x(1:3)));
    f2 = f_MLE_obj(x_new(1:3),x_new(4:6),scale,K,detect_info);
    rho = (f'*f- f2'*f2)/(deltax'*(lambda*diag(diag(H))*deltax+g));
    if rho> e4
        x(4:6) = x(4:6) + deltax(4:6)';
        x(1:3) = f_rotm2lee(f_lee2rotm(deltax(1:3)')*f_lee2rotm(x(1:3)));
        lambda = max(lambda/2,1e-7);
        x_step = norm(deltax);
        if iter==0
            func_step = 10;
        else
            func_step = abs(last_loss - f'*f)/(size(f,1)/2+1-7);
        end
    else
        lambda = min(lambda*3,1e7);
        x_step = 0;
    end
    if max(abs(g))<e1 | max(abs(deltax./x'))<e2 | func_step<1e-6
        break;
    end
    last_loss = f'*f;
    iter = iter + 1;
    disp("iter: "+num2str(iter)+" x_step: "+num2str(x_step)+" cost: "+num2str(f'*f));
end
R = f_lee2rotm(x(1:3));
t = x(4:6)';
TOptm_o = [[R,t];[0,0,0,1]];
end

function [f,H,g]=  f_MLE_obj(rvec,tvec,s,K,detect_info)
R = f_lee2rotm(rvec);
t = tvec';
f=[];
H = zeros([6,6]);
g = zeros([6,1]);
Sigma=[];
K_inv = inv(K);
J_x=[];
d =zeros([6,6]);
for idx =1:size(detect_info,2)
    p_model = double(detect_info{idx}.p_model);
    ui = detect_info{idx}.ui;
    uj = detect_info{idx}.uj;
    n_li = p_model(1:3);
    bar_p_li = p_model(4:6);
    T_i = detect_info{idx}.T_i;
    T_j = detect_info{idx}.T_j;
    Ri = T_i(1:3,1:3);
    ti = T_i(1:3,4);
    Rj = T_j(1:3,1:3);
    tj = T_j(1:3,4);
    for idx2 = 1:size(ui,2)
        u_ik = ui(:,idx2);
        u_jk = uj(:,idx2);
        q_ik = [u_ik;1];
        q_ik = K_inv*q_ik;
        err_ijk = f_obj_ijk(R,t,Ri,ti,Rj,tj,s,K,n_li,bar_p_li,q_ik,u_jk);
        f = [f;err_ijk];
    end
end
if nargout>1
    for idx =1:size(detect_info,2)
        p_model = double(detect_info{idx}.p_model);
        p_cov = double(detect_info{idx}.p_cov);
        px_cov = 1.5*eye(4,4);
        ui = detect_info{idx}.ui;
        uj = detect_info{idx}.uj;
        n_li = p_model(1:3);
        bar_p_li = p_model(4:6);
        T_i = detect_info{idx}.T_i;
        T_j = detect_info{idx}.T_j;
        Ri = T_i(1:3,1:3);
        ti = T_i(1:3,4);
        Rj = T_j(1:3,1:3);
        tj = T_j(1:3,4);
        for idx2 = 1:size(ui,2)
            u_ik = ui(:,idx2);
            u_jk = uj(:,idx2);
            q_ik = [u_ik;1];
            q_ik = K_inv*q_ik;
            err_ijk = f_obj_ijk(R,t,Ri,ti,Rj,tj,s,K,n_li,bar_p_li,q_ik,u_jk);
            J_t_ijk = f_e_jocb_t(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_R_ijk = f_e_jocb_R(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_uik = f_e_jocb_uik(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_ujk = f_e_jocb_ujk();
            J_nijk = f_e_jocb_n(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_barP_ijk = f_e_jocb_barP(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_wk = [J_uik;J_ujk;J_nijk;J_barP_ijk]';
            J_xk =[J_R_ijk;J_t_ijk]';

            sigma = blkdiag(px_cov,p_cov);
            sigma_x = J_wk*sigma*J_wk';
            inv_sigma_x = inv(sigma_x);
            H = H+J_xk'*inv_sigma_x*J_xk;
            g = g-J_xk'*(inv_sigma_x)*err_ijk;
%             H = H+J_xk'*J_xk;
%             g = g-J_xk'*err_ijk;
            J_x = [J_x;J_xk];
            %         H1 = J_x(end-1:end,:)'*J_x(end-1:end,:);
            %         d = d+H1-J_xk'*J_xk;
        end
    end
end
end
