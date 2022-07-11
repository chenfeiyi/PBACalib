function d_Pk_n = f_derivative_Pk_n(R,t,q_k,n_l,bar_p_l)
d_Pk_n = ((R'*t+bar_p_l)/(q_k'*R*n_l) - ...
    ((t'*R+bar_p_l')*n_l*R'*q_k)/((q_k'*R*n_l)^2))*q_k';
end
