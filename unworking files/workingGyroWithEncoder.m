function [x_robot, y_robot, timusamp] = workingGyroWithEncoder(num, gyro_start, encod_start)
% encoder 

encoder_name = ['Encoders', num, '.mat']; 
imu_time = ['imuRaw', num '.mat']; 
load(imu_time);
load(encoder_name);
clear vals;

%gyro_start = getShift(ts);
%encod_start= getShift(Encoders.ts);

imu_times = ts(gyro_start:end);

% angles_name = ['angles' num '.mat'];
% load(angles_name)
imu_name = ['IMUtestdata/conv_imuRaw', num '.mat']; 


load(imu_name);


encoder_counts= Encoders.counts;
encoder_times = Encoders.ts(encod_start:end);


%robot_center = 196.875;
robot_center = 263.525;
robot_width = robot_center*2;
%wheel_diameter = 254;
wheel_diameter = 165.1;
wheel_radius = wheel_diameter/2;
wheel_circum= wheel_diameter*pi;
encoder_clicks = 360;
encoder_rate = 1/40;
gyro_rate = 1/100;
length_per_degree= (wheel_circum/360);
theta(1) = 0;%-pi/8;
%theta(1) = -pi/4;
x(1) = 0;
y(1) = 0;


imu_yaw = vals(:,6);
%imu_yaw = angles23(:,3);
imu_yaw_shift = imu_yaw(gyro_start:end);
imu_yaw_shift(1) = imu_yaw_shift(1) + (theta(1))*100;
imu_new= cumsum(imu_yaw_shift*1/100);


for i =1:length(encoder_counts)-encod_start
    %yaw_angle = imu_gyro(gyro_start+5*i,3)*(1/100);
    distance_right_f = length_per_degree*encoder_counts(1,encod_start+i);
    distance_right_b = length_per_degree*encoder_counts(3,encod_start+i);
    distance_right(i) = (distance_right_f+distance_right_b)/2;
    distance_left_f = length_per_degree*encoder_counts(2,encod_start+i);
    distance_left_b = length_per_degree*encoder_counts(4,encod_start+i);
    distance_left(i) = (distance_left_f+distance_left_b)/2;
    
    %check if robot is going straight or not
    dr = (encoder_counts(1,encod_start+i)+encoder_counts(3,encod_start+i))/2;
    dl = (encoder_counts(2,encod_start+i)+encoder_counts(4,encod_start+i))/2;
    temp_diff= abs(dl-dr); 
    if temp_diff<0.6
        d_corr(i) = 0;
    else
        d_corr(i)=1;
        %d_corr(i)= (distance_right(i) - distance_left(i))/robot_width;
    end
    
    d_center(i) = (distance_right(i) + distance_left(i))/2;
     phi(i)= (distance_right(i) - distance_left(i))/robot_width;
    theta(i+1) = theta(i)+phi(i);
    x(i+1) = x(i) + d_center(i) * cos(theta(i));
    y(i+1) = y(i) + d_center(i) * sin(theta(i));
end

x_robot(1)=0;
y_robot(1)=0;

for i = 1:length(imu_new)/5
    timusamp(i)= imu_new(5*i);
    tencsamp(i)= theta(2*i);
    
     if d_corr(2*i) == 0
         d_theta(i) = imu_new(5*i)-theta(2*i);%-imu_yaw_shift(5*i);
     else
        d_theta(i) = imu_new(5*i)-theta(2*i); 
     end
    dx(i) = x(2*(i+1)) - x(2*i);
    dy(i) = y(2*(i+1)) - y(2*i);
    
    x_robot(i+1) = x_robot(i) + cos(d_theta(i))*dx(i) - sin(d_theta(i))*dy(i);
    y_robot(i+1) = y_robot(i) + sin(d_theta(i))*dx(i) + cos(d_theta(i))*dy(i);
    
    
end
end
% figure; plot(1:length(tencsamp),tencsamp); hold on;
% plot(1:length(timusamp),timusamp, 'r'); 
% 
% figure; plot(1:length(d_theta),d_theta); 

%figure; plot(cumsum(x),cumsum(y))
%figure; plot(x,y); hold on; plot(x_robot,y_robot,'r'); legend('encoder path', 'encoder with gryo path', 'Location', 'SouthEast')



