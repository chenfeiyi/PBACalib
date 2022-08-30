function img_undist= myundistortImage(img_in, K, D)
% Corrects an image for lens distortion.
% K为内参矩阵，用来归一化坐标的，给定的d
% D为径向畸变参数，给定的

IntrinsicMatrix = K';
radialDistortion = [D(1),D(2),D(5)];
tangentialDist = [D(3),D(4)];
imageSize = [size(img_in,1),size(img_in,2)];
principalPoint =[K(1,3),K(2,3)];
cam_param = cameraParameters('IntrinsicMatrix',IntrinsicMatrix,'RadialDistortion',radialDistortion,'TangentialDistortion',tangentialDist);
img_undist = undistortImage(img_in,cam_param); 

end