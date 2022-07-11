function J_n = f_e_jocb_n(R,t,Ri,ti,Rj,tj,scale,K,q_ik,n_li,bar_p_li)
d_Pk_n = f_derivative_Pk_n(R,t,q_ik,n_li,bar_p_li);
d_pjk_pik = f_derivative_pjk_pik(Ri,Rj);
d_e_pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li);
J_n = d_Pk_n*d_pjk_pik*d_e_pjk;
end