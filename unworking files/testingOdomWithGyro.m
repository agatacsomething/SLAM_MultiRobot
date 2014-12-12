% encoder 
encoder_name = 'Encoders23.mat'; 
imu_time = 'imuRaw23.mat'; 
load(imu_time);
clear vals;
imu_times = ts;

imu_name = 'IMUtestdata/conv_imuRaw23.mat'; 

load(encoder_name);
load(imu_name);
encoder_counts= Encoders.counts;
encoder_times = Encoders.ts;
gyro_start = getShift(ts);
encod_start= getShift(encoder_times);


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
length_per_degree= (wheel_circum/180);
theta_enc(1) = 0;
x_enc(1) = 0;
y_enc(1) = 0;
x_robot(1) = 0;
y_robot(1) = 0;
yaw_angle_add(1)=0;

for i =1:(length(encoder_counts)-encod_start-2)/2 -35
    yaw_angle(i) = imu_gyro(gyro_start+5*(i-1),3)*(1/100);
    yaw_angle_add(i+1) = yaw_angle_add(i) + yaw_angle(i);
    
    distance_right_f = length_per_degree*encoder_counts(1,encod_start+2*(i-1));
    distance_right_b = length_per_degree*encoder_counts(3,encod_start+2*(i-1));
    distance_right(i) = (distance_right_f+distance_right_b)/2;
    distance_left_f = length_per_degree*encoder_counts(2,encod_start+2*(i-1));
    distance_left_b = length_per_degree*encoder_counts(4,encod_start+2*(i-1));
    distance_left(i) = (distance_left_f+distance_left_b)/2;
    d_center(i) = (distance_right(i) + distance_left(i))/2;
    phi(i)= (distance_right(i) - distance_left(i))/robot_width;
    
    theta_enc(i+1) = theta_enc(i)+phi(i);
    
    
    x_enc(i+1) = x_enc(i) + d_center(i) * cos(theta_enc(i));
    y_enc(i+1) = y_enc(i) + d_center(i) * sin(theta_enc(i));
    
    dx = x_enc(i+1) - x_enc(i);
    dy = y_enc(i+1) - y_enc(i);
    d_theta = yaw_angle_add(i+1) - theta_enc(i+1);
    
    x_robot(i+1) = x_robot(i) + cos(d_theta)*dx - sin(d_theta)*dy;
    y_robot(i+1) = y_robot(i) + sin(d_theta)*dx + cos(d_theta)*dy;
end

%figure; plot(cumsum(x),cumsum(y))
figure; plot(x_robot,y_robot)



