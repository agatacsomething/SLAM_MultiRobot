% clear all
% close all

function [flippedpartialaccallvoltsminusbias_wsens] = imu2acc (temp_name, show_acc_graphs)

%redo from 2-6

% folder = 'test_loop/';
% name = 'imu07';
%varname = genvarname(name);
%temp_name = load(['/Users/agata/Desktop/project3/' mainfoldername '/' folder '/' file_name]);

%[pathstr, name, ext] = fileparts(file_name) ;
%temp_name= temp.(name);

% 
% figure; plot(1:size(temp_name,1), temp_name(:,1:3))
% hold on
% legend('Ax', 'Ay','Az')

partialaccall = temp_name(:,1:3);
partialaccallvolts = partialaccall*3300/1023;
% figure; plot(1:size(partialaccallvolts,1), partialaccallvolts)
% hold on
% legend('Ax', 'Ay','Az')

avgs= mean(partialaccallvolts(1:5,1:3),1);
newdummy = zeros(size(partialaccall));
newdummy(:,1)= avgs(1);
newdummy(:,2)= avgs(2);
newdummy(:,3)= avgs(3)+300;

partialaccallvoltsminusbias= partialaccallvolts - newdummy;
% figure; plot(1:size(partialaccallvolts,1), partialaccallvoltsminusbias);
% hold on
% legend('Ax', 'Ay','Az')

partialaccallvoltsminusbias_wsens= partialaccallvoltsminusbias/300 ;
% figure; plot(1:size(partialaccallvolts,1), partialaccallvoltsminusbias_wsens);
% hold on
% legend('Ax', 'Ay','Az')


flippedpartialaccallvoltsminusbias_wsens(:,1)= -partialaccallvoltsminusbias_wsens(:,1);
flippedpartialaccallvoltsminusbias_wsens(:,2)= -partialaccallvoltsminusbias_wsens(:,2);
flippedpartialaccallvoltsminusbias_wsens(:,3)= partialaccallvoltsminusbias_wsens(:,3);

if show_acc_graphs ==1
    figure; plot(1:size(partialaccallvolts,1), flippedpartialaccallvoltsminusbias_wsens);
    hold on
    legend('Ax', 'Ay','Az')
    title('Final corrected accelerometer data')
    xlabel('Reading number')
    ylabel('Value in gs')
end
