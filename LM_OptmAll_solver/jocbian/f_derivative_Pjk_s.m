function d_Pjk_s = f_derivative_Pjk_s(Ri,Rj,ti,tj)
d_Pjk_s = (tj-Rj*Ri'*ti)';
end