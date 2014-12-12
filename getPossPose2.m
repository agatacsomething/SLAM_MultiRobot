function [besttheta, new_lidar]=getPossPose2(bwwalls, x_rob,y_rob,x_rob_map,y_rob_map,xis2,yis2,oldtheta)
global MAP
% clear xis3map;
% clear yis3map;

%genPossPose (x_rob,y_rob, xis,yis)
% x_rob=rob4.pose(i,1);
% y_rob=rob4.pose(i,2);

% x_rob_map=x_rob4(i);
% y_rob_map=y_rob4(i);

% x_im = MAP.xmin:MAP.res:MAP.xmax; %x-positions of each pixel of the map
% y_im = MAP.ymin:MAP.res:MAP.ymax; %y-positions of each pixel of the map
% 
% xis=xis2;
% yis=yis2;
% 
% x_range = -.01:.001:.01;
% y_range = -.01:.001:.01;

% xis2=robot{robot_num}.packet{i+1}.hlidar.xs;%-pose(i,1);
% yis2=robot{robot_num}.packet{i+1}.hlidar.ys;%-pose(i,2);

xis3map(:,1) = ceil((xis2(:,1)*1000 - MAP.xmin) ./ MAP.res);
yis3map(:,1) = ceil((yis2(:,1)*1000 - MAP.xmin) ./ MAP.res);



tempmap=zeros(size(bwwalls));


temp_xrob= repmat([x_rob;y_rob],1,length(yis2));

tot_is= [xis2,yis2]';
shift_tot= tot_is-temp_xrob;

xis3maptemp(:,1) = ceil((shift_tot(:,1)*1000 - MAP.xmin) ./ MAP.res);
yis3maptemp(:,1) = ceil((shift_tot(:,2)*1000 - MAP.xmin) ./ MAP.res);

thetas=[-1.25:0.05:1.25]+oldtheta;

for num = 1:length(thetas)
    clear xis3;
    clear yis3; 
    A= [cosd(thetas(num)), sind(thetas(num)); -sind(thetas(num)), cosd(thetas(num))];

    %temp_xrob= repmat([x_rob;y_rob],1,length(shift_yis));

    new_is = temp_xrob+A*shift_tot;

    
    xis3(1,:) = ceil((new_is(1,:)*1000 - MAP.xmin) ./ MAP.res);
	yis3(1,:) = ceil((new_is(2,:)*1000 - MAP.ymin) ./ MAP.res);
    
    %posY2 = [xis3; yis3; zeros(1,length(yis3))];
    inds = sub2ind(size(MAP.map),xis3,yis3);
    tempmap(inds)= 1; 
    
    R(num) = corr2(bwwalls,tempmap);
    %c = map_correlation(int8(log_prob_walls),x_im,y_im,posY2,x_range,y_range);
    %figure; surf(c) 
    %maxes(num) = max(max(c));

end

[val idx]=max(R);

besttheta= thetas(idx);

A= [cosd(thetas(idx)), sind(thetas(idx)); ...
    -sind(thetas(idx)), cosd(thetas(idx))];
new_lidar = temp_xrob+A*shift_tot;

% maxes
% [val idx]=max(maxes)
% thetas(idx)

% figure; imshow(bwwalls)
% hold on; scatter(yis3, xis3,'g.')
% %hold on; scatter(yis3map, xis3map,'y.')
% 
% hold on; plot(y_rob_map, x_rob_map,'y')