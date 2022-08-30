clc;
clear;
addpath("utils/");
K = [897.4566,0,635.4040;
    0,896.7992,375.3149;
    0,0,1];
D = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
img_folder = "/home/cfy/Documents/livoxBACali/data/real/scene4_2/img";
save_folder = "/home/cfy/Documents/livoxBACali/data/real/scene4_2/img_un";
[img_list,imgs_raw] = f_load_data(img_folder,'img');
for idx=1:size(imgs_raw,2)
   img_un = myundistortImage(imgs_raw{idx},K,D);
   save_file = save_folder+"/"+img_list{idx};
   imwrite(img_un,save_file);
end