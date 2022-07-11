function jocb_Rj = f_e_jocb_Rj(R,t,Ri,ti,Rj,tj,scale,K,q_ik,n_li,bar_p_li)
d_Pjk_Rj = f_derivative_Pjk_Rj(R,t,Ri,Rj,ti,tj,scale,q_ik,n_li,bar_p_li);
d_e_Pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li);
jocb_Rj = d_Pjk_Rj*d_e_Pjk;
end