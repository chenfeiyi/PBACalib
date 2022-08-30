function jocb_ti = f_e_jocb_ti(R,t,Ri,ti,Rj,tj,scale,K,q_ik,n_li,bar_p_li)
d_Pjk_ti = f_derivative_Pjk_ti(Ri,Rj,scale);
d_e_Pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li);
jocb_ti = d_Pjk_ti*d_e_Pjk;
end