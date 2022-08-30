function diff_o = f_PxsWithProErr_c(Ti,Tj,K,ui,uj,p_model)

R = Ti(1:3,1:3);
t = Ti(1:3,4);
n = p_model(1:3);
bar_p = p_model(4:6);
K_inv=inv(K);
pts=[];
for idx=1:size(ui,2)
    qk = K_inv*[ui(:,idx);1];
    pt = (t'*R*n+bar_p'*n)/(qk'*R*n)*qk;
    pts =[pts,pt];
end

T1to2 = Tj*inv(Ti);
pts_aft = T1to2(1:3,1:3)*pts+T1to2(1:3,4);
px_aft = K*pts_aft;
px_aft =px_aft(1:2,:)./px_aft(3,:);

diff = px_aft - uj;
diff = vecnorm(diff);
diff_o = diff;

end