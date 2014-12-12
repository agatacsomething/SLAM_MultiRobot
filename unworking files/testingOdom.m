% encoder 
encoder_name = 'Encoders23.mat'; 
imu_time = 'imuRaw23.mat'; 
load(imu_time);
clear vals;
imu_times = ts(63:end);
gyro_start=63-5;
encod_start=37-2;

imu_name = 'IMUtestdata/conv_imuRaw23.mat'; 

load(encoder_name);
load(imu_name);
encoder_counts= Encoders.counts;
encoder_times = Encoders.ts(37:end);
imu_gyro = vals(:,4:6); 

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
length_per_degree= (wheel_circum/330);
theta(1) = pi/8;
%theta(1) = -pi/4;
x(1) = 0;
y(1) = 0;

for i =1:length(encoder_counts)-encod_start
    %yaw_angle = imu_gyro(gyro_start+5*i,3)*(1/100);
    distance_right_f = length_per_degree*encoder_counts(1,encod_start+i);
    distance_right_b = length_per_degree*encoder_counts(3,encod_start+i);
    distance_right(i) = (distance_right_f+distance_right_b)/2;
    distance_left_f = length_per_degree*encoder_counts(2,encod_start+i);
    distance_left_b = length_per_degree*encoder_counts(4,encod_start+i);
    distance_left(i) = (distance_left_f+distance_left_b)/2;
    d_center(i) = (distance_right(i) + distance_left(i))/2;
    phi(i)= (distance_right(i) - distance_left(i))/robot_width;
    theta(i+1) = theta(i)+phi(i);
    x(i+1) = x(i) + d_center(i) * cos(theta(i));
    y(i+1) = y(i) + d_center(i) * sin(theta(i));
end

%figure; plot(cumsum(x),cumsum(y))
figure; plot(-y,x); hold on;



