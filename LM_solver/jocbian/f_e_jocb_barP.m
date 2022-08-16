function d_e_barP = f_e_jocb_barP(R,t,Ri,ti,Rj,tj,scale,K,q_ik,n_li,bar_p_li)
d_e_Pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li);
d_pjk_pik = f_derivative_pjk_pik(Ri,Rj);
d_pik_barP = f_derivative_Pk_barP(R,q_ik,n_li);
d_e_barP = d_pik_barP*d_pjk_pik*d_e_Pjk;
end
