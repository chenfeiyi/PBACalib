function f_plot_match(img1,px1,img2,px2)
h1 = size(img1,1);
w1 = size(img1,2);
h2 = size(img2,1);
w2 = size(img2,2);

h = max(h1,h2);

img_merge = zeros(h,w1+w2,3,'uint8');
img_merge(1:h1,1:w1,:) = img1;
img_merge(1:h2,w1+1:w1+w2,:) = img2;


px2 = px2+[w1,0]';

figure;
hold on;
imshow(img_merge);
hold on;
for idx=1:size(px1,2)
hold on;
plot([px1(1,idx),px2(1,idx)],[px1(2,idx),px2(2,idx)],'o-');
end
hold off;
end