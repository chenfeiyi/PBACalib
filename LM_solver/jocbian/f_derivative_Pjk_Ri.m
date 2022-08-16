% the derivative of Ti on p_jk
function d_Pjk_Ri = f_derivative_Pjk_Ri(R,t,Ri,Rj,ti,tj,scale,q_ik,n_li,bar_p_li)
p_ik = f_obj_pk(R,t,q_ik,n_li,bar_p_li);
d_Pjk_Ri = -f_skew(p_ik)*Ri*Rj'+scale*f_skew(ti)*Ri*Rj';
end
