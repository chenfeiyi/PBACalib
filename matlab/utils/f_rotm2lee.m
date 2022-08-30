function lee_vec_o = f_rotm2lee(R)
quat = rotm2quat(R);
theta = 2*acos(quat(1));
n_vec = quat(2:4)./sin(theta/2);
if theta==0
    lee_vec_o=[0,0,0];
else
    lee_vec_o = theta*n_vec;
end
end