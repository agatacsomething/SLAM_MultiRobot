num = '23';


%% Init of Map
MAP.res   = 0.1*1000; %meters

MAP.xmin  = -10*1000;  %meters
MAP.ymin  = -15*1000;
MAP.xmax  =  25*1000;
MAP.ymax  =  15*1000;


% dimensions of the map
MAP.sizex  = ceil((MAP.xmax - MAP.xmin) / MAP.res + 1); %cells
MAP.sizey  = ceil((MAP.ymax - MAP.ymin) / MAP.res + 1);

MAP.map = zeros(MAP.sizex,MAP.sizey,'int8');
% figure('name','Empty Map');
% imagesc(MAP.map);
%%

encoder_name = ['Encoders', num, '.mat']; 
imu_time = ['imuRaw', num '.mat']; 
load(imu_time);
load(encoder_name);
clear vals;

lidar_name = ['Hokuyo',num,'.mat'];
load(lidar_name);
lidar_times = Hokuyo0.ts; 
lidar_readings = Hokuyo0.ranges;

[lidar_val idx1] = getShift (lidar_times);
[gyro_val  idx2] = getShift(ts);
[encod_val idx3] = getShift(Encoders.ts);

lidar_start = idx1(1);
gyro_start= idx2(2);
encod_start= idx3(1);

lidar_readings= lidar_readings(:,lidar_start:end);

[x_robot, y_robot, thetas] = workingGyroWithEncoder(num, gyro_start, encod_start);
x_robot_new = x_robot(1:end-1);
y_robot_new = y_robot(1:end-1);

lidar_angles = Hokuyo0.angles; 
lidar_reading_new=lidar_readings;

anglespan = 150:950;
%totalpoints = length(anglespan*1863);
totalpoints = length(anglespan)*1863;
ocup_points= zeros(totalpoints, 2);

%lidar_angles = lidar_angles(150:950);
k=0;
tic
for i =1:1863
    for j = anglespan
            angle = (lidar_angles(j) +thetas(i));
            
        
%             if j==180+50
%                 k=k+1;
%                 anglessave(k,1)=(lidar_angles(j) +thetas(i));
%                 anglessave(k,2)=thetas(i);
%                  lidar_reading=lidar_reading_new(j,2*i);
%                  ocup_points(k,1) = (cos(angle)*(lidar_reading*1000))+x_robot_new(i);
%                  ocup_points(k,2) = (sin(angle)*(lidar_reading*1000))+y_robot_new(i);
%             end
           if lidar_reading_new(j,2*i) > .01 && lidar_reading_new(j,2*i) < 5
              k=k+1;

            lidar_reading=lidar_reading_new(j,2*i);
            ocup_points(k,1) = (cos(angle)*(lidar_reading*1000))+x_robot_new(i);
            ocup_points(k,2) = (sin(angle)*(lidar_reading*1000))+y_robot_new(i);

          else
              continue
          end
    end
end

toc

%% check this out
% xis = ceil((ocup_points(:,1) - MAP.xmin) ./ MAP.res);
% yis = ceil((ocup_points(:,2) - MAP.ymin) ./ MAP.res);
%             
% %check the indices and populate the map
% % indGood = (xis > 1) & (yis > 1) & (xis < MAP.sizex) & (yis < MAP.sizey);
% % inds = sub2ind(size(MAP.map),xis(indGood),yis(indGood));
% % MAP.map(inds) = 100;
% 
% %compute correlation
% x_im = MAP.xmin:MAP.res:MAP.xmax; %x-positions of each pixel of the map
% y_im = MAP.ymin:MAP.res:MAP.ymax; %y-positions of each pixel of the map
% figure('name','Filled Map');
% imagesc(MAP.map);

%%
xis = ceil((ocup_points(:,1) - MAP.xmin) ./ MAP.res);
yis = ceil((ocup_points(:,2) - MAP.ymin) ./ MAP.res);
indGood = (xis > 1) & (yis > 1) & (xis < MAP.sizex) & (yis < MAP.sizey);
inds = sub2ind(size(MAP.map),xis(indGood),yis(indGood));

[testa testb] =histc(inds,unique(inds));


k=length(testa); 
newinds = unique(inds); 
testa = testa>30; 

MAP.map(newinds) = testa*100;

figure('name','Filled Map 2');
imagesc(MAP.map);
%%

%rgmyck
%bck
%figure;  %legend('encoder path', 'encoder with gryo path', 'Location', 'SouthEast')

%figure; plot(-ocup_points(76384:end,2),ocup_points(76384:end,1),'r.'); 
%hold on; plot(-y_robot_new,x_robot_new,'k');
figure; plot(-ocup_points(:,2),ocup_points(:,1),'b.');
hold on; plot(-y_robot_new,x_robot_new,'k');