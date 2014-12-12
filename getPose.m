function [x_robot, y_robot, timusamp, th_enc, xencod, yencod] = ...
    getPose(imu_new, encoder_counts, lpd, rw, ...
    start_x, start_y, iter, theta_init, xencod_init, yencod_init)
% encoder 

length_per_degree= lpd; 
robot_width = rw; 
theta(1) = theta_init;
x(1) = xencod_init;
y(1) = yencod_init;

for i =1:2
    %yaw_angle = imu_gyro(gyro_start+5*i,3)*(1/100);
    distance_right_f = length_per_degree*encoder_counts(1,2*iter+i);
    distance_right_b = length_per_degree*encoder_counts(3,2*iter+i);
    distance_right = (distance_right_f+distance_right_b)/2;
    distance_left_f = length_per_degree*encoder_counts(2,2*iter+i);
    distance_left_b = length_per_degree*encoder_counts(4,2*iter+i);
    distance_left = (distance_left_f+distance_left_b)/2;
  
    
    d_center = (distance_right + distance_left)/2;
    phi= (distance_right - distance_left)/robot_width;
    theta(i+1) = theta(i)+phi;
    x(i+1) = x(i) + d_center * cos(theta(i));
    y(i+1) = y(i) + d_center * sin(theta(i));
end

xencod = x(3);
yencod = y(3);
th_enc= theta(3);

% x_robot(1)=start_x;
% y_robot(1)=start_y;

if iter == 0
    timusamp= 0;
else 
    timusamp= imu_new(5*iter);
end

d_theta = timusamp-theta(3);%-imu_yaw_shift(5*i);

dx = x(3) - x(1);
dy = y(3) - y(1);
    
x_robot = start_x + cos(d_theta)*dx - sin(d_theta)*dy;
y_robot = start_y + sin(d_theta)*dx + cos(d_theta)*dy;
    

end
% figure; plot(1:length(tencsamp),tencsamp); hold on;
% plot(1:length(timusamp),timusamp, 'r'); 
% 
% figure; plot(1:length(d_theta),d_theta); 

%figure; plot(cumsum(x),cumsum(y))
%figure; plot(x,y); hold on; plot(x_robot,y_robot,'r'); legend('encoder path', 'encoder with gryo path', 'Location', 'SouthEast')



