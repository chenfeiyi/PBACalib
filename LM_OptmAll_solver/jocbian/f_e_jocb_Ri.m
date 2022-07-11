function jocb_Ri = f_e_jocb_Ri(R,t,Ri,ti,Rj,tj,scale,K,q_ik,n_li,bar_p_li)
d_Pjk_Ri = f_derivative_Pjk_Ri(R,t,Ri,Rj,ti,tj,scale,q_ik,n_li,bar_p_li);
d_e_Pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li);
jocb_Ri = d_Pjk_Ri*d_e_Pjk;
end