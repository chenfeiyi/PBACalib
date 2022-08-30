function [pxs_o,pts3D_o]=f_getPxPoints3D(images,points3D,img_id)
pxs_o=[];
pts3D_o=[];
if ~images.isKey(img_id)
    return;
end

pts_id = images(img_id).point3D_ids;
for idx=1:size(pts_id,1)
    if ~points3D.isKey(pts_id(idx))
        continue;
    end
    if ~points3D(pts_id(idx)).isplane
        continue;
    end
    pxs_o = [pxs_o,images(img_id).xys(idx,:)'];
    pts3D_o = [pts3D_o,points3D(pts_id(idx)).xyz];
end
end