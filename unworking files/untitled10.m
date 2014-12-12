%Encoders.ts(37)-ts(63)

time_unix = Encoders.ts(37:130); % example time
%time_unix = ts(64:130);
time_reference = datenum('1970', 'yyyy'); 
time_matlab = time_reference + time_unix / 8.64e7;
time_matlab_string = datestr(time_matlab, 'yyyymmdd HH:MM:SS.FFF')