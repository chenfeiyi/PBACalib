function wirte_model(cameras,images,points3D,file_path)

if numel(file_path) > 0 && file_path(end) ~= '/'
    file_path = [file_path '/'];
end
write_camera(cameras,[file_path,'cameras.txt']);
write_images(images,[file_path,'images.txt']);
write_point3D(points3D,[file_path,'points3D.txt']);

end

function write_camera(cameras,file_name)
fid = fopen(file_name,'w');
head1 = '# Camera list with one line of data per camera:';
head2 = '# CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]';
head3 = ['# Number of cameras: ' int2str(cameras.Count)];
fprintf(fid,'%s\n',head1);
fprintf(fid,'%s\n',head2);
fprintf(fid,'%s\n',head3);
cam_ids = keys(cameras);
for idx=1:cameras.Count
    ss = [int2str(cam_ids{idx}) ' '  cameras(cam_ids{idx}).model ' '];
    ss = [ss int2str(cameras(cam_ids{idx}).width) ' ' int2str(cameras(cam_ids{idx}).height)];
    param_s='';
    for idx2=1:size(cameras(cam_ids{idx}).params,1)
        param_s = [param_s ' ' num2str(cameras(cam_ids{idx}).params(idx2))];
    end
    ss = [ss  param_s];
    fprintf(fid,'%s\n',ss);
end

fclose(fid);
end

function write_images(images,file_path)
fid = fopen(file_path,'w');
head1 = '# Image list with two lines of data per image:';
head2 = '#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME';
head3 = '#   POINTS2D[] as (X, Y, POINT3D_ID)';
head4 = ['# Number of images: ' int2str(images.Count) ', mean observations per image:'];

fprintf(fid,'%s\n',head1);
fprintf(fid,'%s\n',head2);
fprintf(fid,'%s\n',head3);
fprintf(fid,'%s\n',head4);
img_keys = keys(images);
for idx=1:images.Count
    img_id = img_keys{idx};
    ss = [int2str(img_id) ' '];
    ss = [ss  num2str(rotm2quat(images(img_id).R)) ' ' num2str( images(img_id).t') ' '];
    ss = [ss  int2str(images(img_id).camera_id) ' ' images(img_id).name];
    fprintf(fid,'%s\n',ss);
    xys = images(img_id).xys;
    pt_id = images(img_id).point3D_ids;
    sub_ss = '';
    for idx2 = 1:size(xys,1)
        sub_ss= [sub_ss  ' ' num2str(xys(idx2,:)) ' ' int2str(pt_id(idx2))];
    end
    fprintf(fid,'%s\n',sub_ss);
end
fclose(fid);
end

function write_point3D(points3D,file_path)
fid = fopen(file_path,'w');
head1 = '# 3D point list with one line of data per point:';
head2 = '#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)';
head3 = ['# Number of points: ' int2str(points3D.Count) ' mean track length:'];

fprintf(fid,'%s\n',head1);
fprintf(fid,'%s\n',head2);
fprintf(fid,'%s\n',head3);

pt_keys = keys(points3D);
for idx=1:points3D.Count
    pt3D = points3D(pt_keys{idx});
    ss = [int2str(pt3D.point3D_id) ' ' num2str(pt3D.xyz') ' ' int2str(pt3D.rgb') ' '];
    ss = [ss num2str(pt3D.error)];
    sub_ss = '';
    for idx2=1:size(pt3D.track,1)
        sub_ss = [sub_ss ' ' int2str(pt3D.track(idx2,:))];
    end
    ss = [ss  sub_ss];
    fprintf(fid,'%s\n',ss);
end
fclose(fid);
end
