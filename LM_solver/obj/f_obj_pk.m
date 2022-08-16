function p_k = f_obj_pk(R,t,q_k,n_l,bar_p_l)
p_k = ((t'*R*n_l+bar_p_l'*n_l)/(q_k'*R*n_l) )* q_k;
end