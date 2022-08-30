function Sigma_n_q = f_calcPlaneCovariance(U,Lambda,pts,e_d,e_theta)
% desc: refer to "Efficient and Probabilistic 
% Adaptive Voxel Mapping for Accurate Online LiDAR Odometry" for the theory
Sigma_n_q=zeros([6,6]);
q = double(mean(pts,2));
num = size(pts,2);
u1 = U(:,1);
u2 = U(:,2);
u3 = U(:,3);
n = u3;
lambda1 = Lambda(1,1);
lambda2 = Lambda(2,2);
lambda3 = Lambda(3,3);
Sigma_d = e_d*e_d;
Sigma_dir = [sind(e_theta)^2,0;0,sind(e_theta)^2];
for idx=1:size(pts,2)
    pt = double(pts(:,idx));
    dir = pt./norm(pt);
    depth = norm(pt);

    
    if dir(3)==0
        base_vector1 = [0,1,0]';
    else
        base_vector1 = [1,1,-(dir(1)+dir(2))/dir(3)]';
    end
    base_vector1 = base_vector1./norm(base_vector1);
    base_vector2 = cross(base_vector1,dir);
    base_vector2 = base_vector2./norm(base_vector2);
    N = [ base_vector1, base_vector2];
    A_i = [dir, -depth*f_skew(dir)*N];

    Sigma_p_i = A_i*[Sigma_d,0,0;
                  [[0,0]',Sigma_dir]]*A_i';
    F13 = (pt-q)'*(u1*n'+n*u1')/(num*(lambda3-lambda1));
    F23 = (pt-q)'*(u2*n'+n*u2')/(num*(lambda3-lambda2));
    F33 = [0,0,0];
    d_n_pi = U*[F13;F23;F33];
    d_q_pi = diag([1/num,1/num,1/num]);
    J_f_pi = [d_n_pi;d_q_pi];
    Sigma_n_q_i = J_f_pi*Sigma_p_i*J_f_pi';
    if sum(isnan(Sigma_n_q_i))>0
        continue;
    end
    Sigma_n_q = Sigma_n_q + Sigma_n_q_i;

end

end