function [change_cols_gyro] = imu2gyro (file_name, show_gyro_graphs)


partialgyroall = file_name(:,4:6);
partialgyroallvolts = partialgyroall*3300/1023;
 figure; plot(1:length(partialgyroall), partialgyroall(:,1))
 hold on
 legend('\omega_x', '\omega_y','\omega_z')
 
%  figure; plot(1:length(partialgyroallvolts), partialgyroall(:,1))
%  hold on
%  legend('\omega_x', '\omega_y','\omega_z')

avgs= mean(partialgyroallvolts(1:50, 1:3),1);
newdummy = zeros(size(partialgyroall));
newdummy(:,1)= avgs(1);
newdummy(:,2)= avgs(2);
newdummy(:,3)= avgs(3);

partialgyroallvoltsminusbias= partialgyroallvolts - newdummy;
% figure; plot(1:length(partialgyroallvoltsminusbias), partialgyroallvoltsminusbias(:,1));
% hold on
% legend('\omega_z', '\omega_x','\omega_y')

partialgyroallvoltsminusbias_wsens= (partialgyroallvoltsminusbias/3.33)*(pi/180);
% figure; plot(1:length(partialgyroallvoltsminusbias_wsens), partialgyroallvoltsminusbias_wsens(:,1));
% hold on
% legend('\omega_z', '\omega_x','\omega_y')


change_cols_gyro = [partialgyroallvoltsminusbias_wsens(:,2) partialgyroallvoltsminusbias_wsens(:,3) partialgyroallvoltsminusbias_wsens(:,1)];

% figure; plot(1:length(change_cols_gyro), change_cols_gyro(:,3));
% hold on
% legend('\omega_z', '\omega_x','\omega_y')

if show_gyro_graphs ==1
    figure; plot(1:size(file_name,1), change_cols_gyro(:,3));
    hold on
    legend('\omega_x', '\omega_y','\omega_z')
    title('Final corrected gyroscope data')
    xlabel('Reading number')
    ylabel('Rotation around axis in radians from last reading')
end
