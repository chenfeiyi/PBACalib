function [file_list,varargout]=load_data(folder_path,varargin)


data_list = dir(folder_path);
data_list = data_list(3:end);

file_list=[];

file_num=[];
for idx = 1:size(data_list,1)
    if data_list(idx).isdir
        continue;
    end
    file_name = data_list(idx).name;
    sub_str = split(file_name,'.');
    file_num = [file_num,str2num(sub_str{1})];
    file_list = [file_list;string(file_name)];
end
[B,I] = sort(file_num);
file_list = file_list(I);
data_raw={};
if nargin==2 && nargout==2
    data_type=varargin{1};
    if data_type=="pcd"
        for idx=1:size(file_list,1)
            disp("load pcd: "+file_list(idx,:));
            data = pcread(folder_path+"/"+file_list(idx,:));
%             data = pcdownsample(data,'gridAverage',0.05);
            data_raw{idx} = data.Location()';
        end
    elseif data_type=="img"
        for idx=1:size(file_list,1)
            disp("load image: "+file_list(idx,:));
            data_raw{idx} = imread(folder_path+"/"+file_list(idx,:));
        end

    end
    varargout{1} = data_raw;
end

end
