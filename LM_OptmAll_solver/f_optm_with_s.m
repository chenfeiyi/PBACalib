% clc;
% clear;
% load("../detect.mat");
% addpath("jocbian/");
% addpath("obj/");
% s = double(detect_info{1}.scale);
% [TOptm_o,s_optm]=f_optm_with_s2(detect_info,T1,K,s)
function [TOptm_o,s_optm]=f_optm_with_s(detect_info,TInit,K,s)
iter_max = 100;
x_step_min = 1e-6;
func_step_min = 1e-3;
R = TInit(1:3,1:3);
t = TInit(1:3,4);
x0 = [f_rotm2lee(R),t',s];
x = x0;
iter =0;
x_step = 10;
last_loss=0;
func_step = 10;
while x_step>x_step_min && iter<iter_max && func_step>func_step_min
    [f,J_x] = f_obj(x(1:3),x(4:6),x(7),K,detect_info);
    H = J_x'*J_x;
    g = -J_x'*f;
    deltax = inv(H)*g;
    x(4:7) = x(4:7) + deltax(4:7)';
    x(1:3) = f_rotm2lee(f_lee2rotm(deltax(1:3)')*f_lee2rotm(x(1:3)));

    x_step = norm(deltax);
    if iter==0
        func_step = 10;
    else
        func_step = abs(last_loss - f'*f);
    end
    last_loss = f'*f;
    iter = iter + 1;
    disp("iter: "+num2str(iter)+" x_step: "+num2str(x_step)+" cost: "+num2str(f'*f));
end
R = f_lee2rotm(x(1:3));
t = x(4:6)';
s = x(7);
TOptm_o = [[R,t];[0,0,0,1]];
s_optm = s;
end


function [f,J_x]=  f_obj(rvec,tvec,s,K,detect_info)
R = f_lee2rotm(rvec);
t = tvec';
f=[];
K_inv = inv(K);
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

J_x = [];
if nargout > 1 % gradient required
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
            J_t_ijk = f_e_jocb_t(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_R_ijk = f_e_jocb_R(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_s_ijk = f_e_jocb_s(R,t,Ri,ti,Rj,tj,s,K,q_ik,n_li,bar_p_li);
            J_x =[J_x;[J_R_ijk;J_t_ijk;J_s_ijk]'];
        end
    end
    J_x = double(J_x);
end

end
