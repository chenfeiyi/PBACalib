T1=[0.028787,-0.999536,0.00994784,-0.0544361
0.0212711,-0.00933715,-0.99973,0.0823463
0.999359,0.0289908,0.0209925,0.0832728
0,0,0,1];
T2=[0.0267615,-0.999584,0.0107452,-0.0114978
0.0254952,-0.0100631,-0.999624,0.0352374
0.999317,0.0270254,0.0252153,0.038581
0,0,0,1];
T_ours = [0.0277   -0.9996    0.0108   -0.0138
        0.0229   -0.0101   -0.9997    0.0814
        0.9994    0.0279    0.0226   -0.0106
             0         0         0    1.0000];
img = imread("/media/cfy/4fca2181-366c-4c91-9531-c48ab7e2a252/home/ramlab/Documents/cali_ws/data/1.jpg");
K = [897.4566,0,635.4040;
    0,896.7992,375.3149;
    0,0,1];
D = [-0.4398 0.2329 -0.0011 2.0984e-04 -0.0730];
img_un = myundistortImage(img,K,D);

pc_raw = pcread("/media/cfy/4fca2181-366c-4c91-9531-c48ab7e2a252/home/ramlab/Documents/cali_ws/data/1.pcd");
pc_raw = pcdownsample(pc_raw,"gridAverage",0.02);
% img_pro = pt_project_depth2image(T1,K,pc_raw.Location()',img_un);
img_pro = f_pro_livox2cam_i(pc_raw.Location()',img_un,K,T1,pc_raw.Intensity');
imshow(img_pro)
figure;
img_pro = f_pro_livox2cam_i(pc_raw.Location()',img_un,K,T2,pc_raw.Intensity');
imshow(img_pro)
figure;
img_pro = f_pro_livox2cam_i(pc_raw.Location()',img_un,K,T3,pc_raw.Intensity');
imshow(img_pro)