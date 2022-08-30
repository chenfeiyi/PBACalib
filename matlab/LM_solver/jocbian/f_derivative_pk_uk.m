function d_pk_uk = f_derivative_pk_uk(R,t,K,n_l,bar_p_l,qk)
d_pk_qk = ((t'*R*n_l+bar_p_l'*n_l)/(qk'*R*n_l))'*eye(3)-...
            (t'*R*n_l+bar_p_l'*n_l)/((qk'*R*n_l)^2) * R*n_l*qk';
d_pk_uk = [1,0,0;0,1,0]*inv(K)'*d_pk_qk;
end