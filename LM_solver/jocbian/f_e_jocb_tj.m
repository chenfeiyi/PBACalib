function jocb_tj = f_e_jocb_tj(R,t,Ri,ti,Rj,tj,scale,K,q_ik,n_li,bar_p_li)
d_Pjk_tj = f_derivative_Pjk_tj(scale);
d_e_Pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li);
jocb_tj = d_Pjk_tj*d_e_Pjk;
end
