% My noise model for encoder values i.e. for x/y locations 
% of the robot pose is given by :
enc_constt = 0.005;
enc_prop = std(vals(:,6));
encoder_noise = (enc_prop*theta + enc_constt)*randn(1,1);

% where 
% enc_prop = standard deviation of all the delta theta
% values i.e. change in orientation values for all states
% enc_constt = a constant value that is added to the noise
% (set to 0.005)

% My noise model for orientation of the robot is given by : 

gyro_prop= std(vals(:,6));
gyro_constt = pi/150;
gyro_noise = (gyro_prop*theta + gyro_constt)*randn(1,1);

%where gyro_prop = standard deviation of all the delta theta 
% values i.e. change in orientation values for all states
% gyro_constt = a constant value that is added to the noise 
%(set to pi/150)








%convert from meters to cells
xis = ceil((xs1 - MAP.xmin) ./ MAP.res);
yis = ceil((ys1 - MAP.ymin) ./ MAP.res);

%check the indices and populate the map
indGood = (xis > 1) & (yis > 1) & (xis < MAP.sizex) & (yis < MAP.sizey);
inds = sub2ind(size(MAP.map),xis(indGood),yis(indGood));
MAP.map(inds) = 100;

%compute correlation
x_im = MAP.xmin:MAP.res:MAP.xmax; %x-positions of each pixel of the map
y_im = MAP.ymin:MAP.res:MAP.ymax; %y-positions of each pixel of the map

x_range = -1:0.05:1;
y_range = -1:0.05:1;

c = map_correlation(MAP.map,x_im,y_im,Y(1:3,:),x_range,y_range);