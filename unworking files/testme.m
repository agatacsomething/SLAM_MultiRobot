
gyro_start=63;
imu_yaw = vals(:,6);
imu_yaw_shift = imu_yaw(gyro_start:end);
imu_yaw_shift(1) = imu_yaw_shift(1) + (pi/8)*100;
imu_new= cumsum(imu_yaw_shift*1/100);