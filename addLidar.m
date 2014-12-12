num = '23';
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

%lidar_angles = lidar_angles(150:950);
k=0;
tic
for i =1:1863
    for j = [160:200, 880:920]
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
%rgmyck
%bck
%figure;  %legend('encoder path', 'encoder with gryo path', 'Location', 'SouthEast')

%figure; plot(-ocup_points(76384:end,2),ocup_points(76384:end,1),'r.'); 
%hold on; plot(-y_robot_new,x_robot_new,'k');
figure; plot(-ocup_points(:,2),ocup_points(:,1),'b.');
hold on; plot(-y_robot_new,x_robot_new,'k');