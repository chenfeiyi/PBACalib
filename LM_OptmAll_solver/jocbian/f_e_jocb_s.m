function J_s = f_e_jocb_s(R,t,Ri,ti,Rj,tj,scale,K,q_ik,n_li,bar_p_li)
d_Pjk_s = f_derivative_Pjk_s(Ri,Rj,ti,tj);
d_e_Pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li);
J_s = d_Pjk_s*d_e_Pjk;
end