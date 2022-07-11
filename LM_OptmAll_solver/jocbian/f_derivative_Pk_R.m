function d_Pk_R = f_derivative_Pk_R(R,t,q_k,n_l,bar_p_l)
item1 = f_skew(R*n_l);
item2 = t/(q_k'*R*n_l) - t'*R*n_l*q_k/((q_k'*R*n_l).^2)-bar_p_l'*n_l*q_k/((q_k'*R*n_l).^2);
d_Pk_R = item1*item2*q_k';
end