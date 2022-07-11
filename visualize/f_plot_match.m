function img_merge=plot_match(img1,px1,img2,px2)
h1 = size(img1,1);
w1 = size(img1,2);
h2 = size(img2,1);
w2 = size(img2,2);

h = max(h1,h2);

img_merge = zeros(h,w1+w2,3,'uint8');
img_merge(1:h1,1:w1,:) = img1;
img_merge(1:h2,w1+1:w1+w2,:) = img2;


px2 = px2+[w1,0]';

for idx=1:size(px1,2)
    img_merge = insertText(img_merge,px1(:,idx)',idx,'BoxOpacity',0,'TextColor','green');
    img_merge = insertText(img_merge,px2(:,idx)',idx,'BoxOpacity',0,'TextColor','green');
end
img_merge = insertMarker(img_merge,px1',"x-mark","Color","red");
img_merge = insertMarker(img_merge,px2',"x-mark","Color","red");
end