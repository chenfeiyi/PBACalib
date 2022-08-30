function p_jk = f_obj_pjk(R,t,Ri,ti,Rj,tj,scale,n_li,bar_p_li,q_ik)
p_ik = f_obj_pk(R,t,q_ik,n_li,bar_p_li);
Rij = Rj*Ri';
tij = tj-Rj*Ri'*ti;
p_jk = Rij*p_ik+scale*tij;
end