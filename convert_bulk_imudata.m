%mkdir('IMUtestdata')

mainfoldername = 'proj4/';
getnames = dir(fullfile(mainfoldername));
file_names = {getnames(13:17).name}
show_graphs =0;


%file_names = {getnames(16).name}
%for i = 1: length(foldernames)
%     filenames_all = dir(fullfile(['/Users/agata/Desktop/project4/' mainfoldername '/' foldernames{i}]));
%     file_names = {filenames_all(3:12).name};
    
    for j = 1:size(file_names,2)
        save_file_name = ['conv_' file_names{j}];
        temp_name = load(['/Users/agata/Desktop/project4/' mainfoldername '/' file_names{j}]);
        temp_name = temp_name.vals';
        [acc_data] = imu2acc(temp_name, show_graphs);
        [gyro_data] = imu2gyro(temp_name, show_graphs);
        vals= [acc_data gyro_data];
        cd IMUtestdata
        save([save_file_name], 'vals')
        cd ..
    end
    
%end