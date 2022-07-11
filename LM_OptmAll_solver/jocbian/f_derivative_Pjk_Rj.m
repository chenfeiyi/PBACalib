% the derivative of Tj on p_jk
function d_Pjk_Rj = f_derivative_Pjk_Rj(R,t,Ri,Rj,ti,tj,scale,q_ik,n_li,bar_p_li)
p_ik = f_obj_pk(R,t,q_ik,n_li,bar_p_li);
d_Pjk_Rj = f_skew(Rj*Ri'*p_ik)-scale*f_skew(Rj*Ri'*ti);
end
