function img_out = f_pt_project_depth2image(transformation,K,pts_in,img_rgb,color)

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
map = colormap(jet(256))*256;
% map =[    62    39   169;...
%     68    52   207;...
%     72    69   234;...
%     72    88   249;...
%     66   107   255;...
%     48   128   252;...
%     44   145   240;...
%     35   162   229;...
%     21   176   217;...
%      1   187   197;...
%     38   194   174;...
%     59   202   147;...
%     97   205   114;...
%    145   203    77;...
%    191   196    46;...
%    229   188    44;...
%    255   191    60;...
%    252   211    47;...
%    246   232    37;...
%    250   252    21];
map = map([256:-1:1],:);
pts_valid(3,:) =(pts_valid(3,:))./0.05;
idx = find(pts_valid(3,:)>256);
pts_valid(3,idx) = 256;
idx = find(pts_valid(3,:)<1);
pts_valid(3,idx) = 1;
for index = 1: size(pro_pt,2)
%     pts_x = [-1,-1,-1,0,0,0,1,1,1];
%     pts_y = [-1,0,1,-1,0,1,-1,0,1];
    pts_x = [-2,-2,-2,-2,-2 -1,-1,-1,-1,-1,0,0,0,0,0,1,1,1,1,1];
    pts_y = [-2,-1,0,1,2,-2,-1,0,1,2,-2,-1,0,1,2,-2,-1,0,1,2];
    pixel_xy = [pts_x;pts_y];
    pixel_xy = pixel_xy + pro_pt(:,index);
%     pixel_xy = pro_pt(:,index);
    
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