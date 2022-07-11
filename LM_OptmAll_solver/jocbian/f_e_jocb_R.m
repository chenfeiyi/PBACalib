function jocb_R = f_e_jocb_R(R,t,Ri,ti,Rj,tj,scale,K,q_ik,n_li,bar_p_li)
d_Pik_R = f_derivative_Pk_R(R,t,q_ik,n_li,bar_p_li);
d_Pjk_Pik = f_derivative_pjk_pik(Ri,Rj);
d_e_Pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li);
jocb_R = d_Pik_R*d_Pjk_Pik*d_e_Pjk;
end