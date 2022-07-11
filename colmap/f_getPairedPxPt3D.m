function  [pxs1_o,pxs2_o,pts_o]=f_getPairedPxPt3D(images,points3D,img_id1,img_id2)
pxs1_o=[];
pxs2_o=[];
pts_o=[];
img_id1 = int64(img_id1);
img_id2 = int64(img_id2);

if ~images.isKey(img_id1) || ~images.isKey(img_id2)
    return;
end
pts_val = values(points3D);
pts_keys = keys(points3D);
for idx=1:points3D.Count
    if ~points3D(pts_keys{idx}).isplane
        continue;
    end
    pt_tr = pts_val{idx}.track;
    if ismember(img_id1,pt_tr(:,1)) && ismember(img_id2,pt_tr(:,1))
        idx1 = find(pt_tr(:,1)==img_id1);
        idx2 = find(pt_tr(:,1)==img_id2);
        pts_id1 = pt_tr(idx1,2);
        pts_id2 = pt_tr(idx2,2);
        if size(pts_id2,1)>1 ||size(pts_id2,2)>1 || size(pts_id1,1)>1 ||size(pts_id1,2)>1
            continue;
        end
        pxs1_o = [pxs1_o,images(img_id1).xys(pts_id1,:)'];
        pxs2_o = [pxs2_o,images(img_id2).xys(pts_id2,:)'];
        pts_o = [pts_o,pts_val{idx}.xyz];
    end
end

end
