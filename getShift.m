function [idx_val idx] = getShift (ts)

imu_timestamps = str2num(datestr(ts(1:200)/86400000 + 719529, 'ssfff'));
imu_timestamps_shift(1) = 0;
imu_timestamps_shift(2:200) = imu_timestamps(1:199);
[valhere1 idx1] = min(imu_timestamps_shift(2:end)'==imu_timestamps(2:end));
[valhere2 idx2] = min(imu_timestamps_shift(idx1+2:end)'==imu_timestamps(idx1+2:end));
%imu_timestamps(idx)
idx = [idx1, idx2];
idx_val = [imu_timestamps(idx1), imu_timestamps(idx2)];

%start_idx = idx(1)+1;

