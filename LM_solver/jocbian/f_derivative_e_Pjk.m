function d_e_Pjk = f_derivative_e_Pjk(R,t,Ri,Rj,ti,tj,scale,K,q_ik,n_li,bar_p_li)
p_jk = f_obj_pjk(R,t,Ri,ti,Rj,tj,scale,n_li,bar_p_li,q_ik);

fx = K(1,1);
fy = K(2,2);
cx = K(1,3);
cy = K(2,3);

X = p_jk(1);
Y = p_jk(2);
Z = p_jk(3);

d_e_Pjk = [fx/Z,          0;...
    0,           fy/Z;...
    -fx*X/(Z*Z),   -fy*Y/(Z*Z)];
end