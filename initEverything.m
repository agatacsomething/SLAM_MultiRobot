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
stdv= std(vals(:,6));

encoder_counts= Encoders.counts;
encoder_counts= encoder_counts(:, encod_start:end);
%encoder_times = Encoders.ts(encod_start:end);


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