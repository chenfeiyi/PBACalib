function R = f_lee2rotm(leevec)
if norm(leevec)==0
    R=eye(3);
else
%     theta = norm(leevec);
%     a = leevec/theta;
%     R = cos(theta)*eye(3)+(1-cos(theta))*a*a'+sin(theta)*f_skew(a);%the
%     optimization will crash
    leevec_skew = [0,-leevec(3),leevec(2);...
               leevec(3),0,-leevec(1);...
               -leevec(2),leevec(1),0];
    R = expm(leevec_skew);
end
end