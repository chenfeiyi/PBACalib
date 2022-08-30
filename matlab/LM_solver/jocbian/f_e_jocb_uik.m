function J_uik = f_e_jocb_uik(R,t,Ri,ti,Rj,tj,scale,K,q_ik,n_li,bar_p_li)
d_pk_uk = f_derivative_pk_uk(R,t,K,n_li,bar_p_li,q_ik);
d_pjk_pik = f_derivative_pjk_pik(Ri,Rj);
d_e_Pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li);
J_uik = d_pk_uk*d_pjk_pik*d_e_Pjk;
end