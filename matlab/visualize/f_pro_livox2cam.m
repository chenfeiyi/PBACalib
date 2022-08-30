function  img_out = pro_livox2cam(pts_in,img,K,T)


map = [0    0.4470    0.7410
    0.8500    0.3250    0.0980
    0.9290    0.6940    0.1250
    0.4940    0.1840    0.5560
    0.4660    0.6740    0.1880
    0.3010    0.7450    0.9330
    0.6350    0.0780    0.1840]*255;
img_out=img;
color_idx_arr = [];
for idx = 1:size(pts_in,2)
%     [models,inliners] = plane_ransac(pts_in,0.03);
%     pts_cur = pts_in(:,inliners);
%     pts_in = pts_in(:,~inliners);
    pts_cur = pts_in{idx};
    flag=1;
    while flag
        color_idx = randi(size(map,1));
        idx_find = find(color_idx_arr==color_idx);
        if size(idx_find,2)~=0 && size(color_idx_arr,2)<size(map,1)
            flag=1;
        else
            flag=0;
        end
    end
    color_idx_arr=[color_idx_arr,color_idx];
    color = map(color_idx,:);
    img_out = pt_project_depth2image(T,K,pts_cur,img_out,color,1);
end


end

function img_out = pt_project_depth2image(transformation,K,pts_in,img_rgb,color,isbold)

img_out = img_rgb;

pts_in = [pts_in;ones(1,size(pts_in,2))];
pts_trans = transformation*pts_in;
idx = find(pts_trans(3,:)>0);
pts_valid = pts_trans(1:3,idx);
pro_pt = K*pts_valid;
pro_pt = pro_pt./pro_pt(3,:);
pro_pt = round(pro_pt(1:2,:));
idx = find(pro_pt(1,:)>0 & pro_pt(1,:)<size(img_rgb,2) & pro_pt(2,:)>0 & pro_pt(2,:)<size(img_rgb,1));
pro_pt = pro_pt(:,idx);
pts_valid = pts_valid(:,idx);
% map = uint8(colormap(parula(20))*256);
if nargin<5
    map = colormap(jet)*256;
    map = map([256:-1:1],:);
end
pts_valid(3,:) =(pts_valid(3,:))./0.1;
idx = find(pts_valid(3,:)>256);
pts_valid(3,idx) = 256;
idx = find(pts_valid(3,:)<1);
pts_valid(3,idx) = 1;
for index = 1: size(pro_pt,2)
    if isbold == 3
        pts_x = [-2,-2,-2,-2,-2 -1,-1,-1,-1,-1,0,0,0,0,0,1,1,1,1,1];
        pts_y = [-2,-1,0,1,2,-2,-1,0,1,2,-2,-1,0,1,2,-2,-1,0,1,2];
        
    elseif isbold==2
        pts_x = [-1,-1,-1,0,0,0,1,1,1];
        pts_y = [-1,0,1,-1,0,1,-1,0,1];
    else
        pts_x = [0];
        pts_y =[0];
    end
 
    pixel_xy = [pts_x;pts_y];
    pixel_xy = pixel_xy + pro_pt(:,index);
    
    if nargin==4
        if pro_pt(2,index) == size(img_rgb,1) || pro_pt(2,index)<3 || pro_pt(1,index) == size(img_rgb,2) || pro_pt(1,index)<3
            img_out(pro_pt(2,index),pro_pt(1,index),:) = map(round(pts_valid(3,index)),:)';
            
        else
            for idx = 1: size(pixel_xy,2)
                img_out(pixel_xy(2,idx),pixel_xy(1,idx),:) = map(round(pts_valid(3,index)),:)';
            end
        end
    else
        if pro_pt(2,index) == size(img_rgb,1) || pro_pt(2,index)<3 || pro_pt(1,index) == size(img_rgb,2) || pro_pt(1,index)<3
            img_out(pro_pt(2,index),pro_pt(1,index),:) = color';
            
        else
            for idx = 1: size(pixel_xy,2)
                img_out(pixel_xy(2,idx),pixel_xy(1,idx),:) = color';
            end
        end
    end
end

end