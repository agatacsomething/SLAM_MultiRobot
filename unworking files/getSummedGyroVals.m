function [gyro_vals_out] = getSummedGyroVals(gyro_vals_in, gyro_shift, add_sum)

total_shift= gyro_shift-add_sum;

%for i =1:size(gyro_vals_in,2)
    gyro_vals_shift = gyro_vals_in(total_shift:end);
%end

total_here=floor(size(gyro_vals_shift,1)/5)


    i=1;
for j = 1:total_here
        
    gyro_vals_out(j)= gyro_vals_shift(i) + gyro_vals_shift(i+1)...
        + gyro_vals_shift(i+2) + gyro_vals_shift(i+3)...
        + gyro_vals_shift(i+4);
        i =i+5;
end
