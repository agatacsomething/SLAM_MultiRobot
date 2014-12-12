function [encoder_counts_out] = getSummedEncoderCounts(encoder_counts_in, shift_time, add_sum)

total_shift= shift_time-add_sum;

for i =1:size(encoder_counts_in,1)
    encoder_counts_shift(i,:) = encoder_counts_in(i, total_shift:end);
end

total_here=floor(size(encoder_counts_shift,2)/2);

for k = 1:size(encoder_counts_shift,1)
    i=1;
    for j = 1:total_here
        
        encoder_counts_out(k,j) = encoder_counts_shift(k,i) + encoder_counts_shift(k,i+1);
        i =i+2;
    end
end