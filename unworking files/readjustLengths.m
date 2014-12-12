function [yaw_out, encod_out] = readjustLengths( ts, encoder_times, imu_yaw_angles, encoder_counts)


gyro_start = getShift(ts);
encod_start= getShift(encoder_times);

encoder_counts_adj = getSummedEncoderCounts(encoder_counts, encod_start, 2);
yaw_angles_adj = getSummedGyroVals(imu_yaw_angles, gyro_start, 5);


size_yaw = size(yaw_angles_adj,2);
size_encoder = size(encoder_counts_adj,2);

if size_yaw>size_encoder
    yaw_out = yaw_angles_adj(:,1:size_encoder);
    encod_out =encoder_counts_adj;
else
    yaw_out = yaw_angles_adj;
    encod_out =encoder_counts_adj(:,1:size_yaw);
end
