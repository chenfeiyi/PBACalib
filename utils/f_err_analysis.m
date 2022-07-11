function [angle_err,t_err] = f_err_analysis(TGt,T)
deltaT = inv(TGt)*T;
deltaQ = rotm2quat(deltaT(1:3,1:3));
angle_err=abs(2*acosd(deltaQ(1)));
t_err = norm(TGt(1:3,4)-T(1:3,4));
end