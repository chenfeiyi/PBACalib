function d_Pk_t = f_derivative_Pk_t(R,q_k,n_l)
d_Pk_t = R*n_l*q_k'/(q_k'*R*n_l);
end
