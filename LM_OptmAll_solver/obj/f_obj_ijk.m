function err_ij = f_obj_ijk(R,t,Ri,ti,Rj,tj,scale,K,n_li,bar_p_li,q_ik,u_jk)

p_jk = f_obj_pjk(R,t,Ri,ti,Rj,tj,scale,n_li,bar_p_li,q_ik);

err_ij = K*p_jk;
err_ij = err_ij(1:2)./err_ij(3) - u_jk;

end