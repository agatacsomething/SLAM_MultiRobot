function [new_is]= rotatedLidar(rob4, rob4_new, ptd,xis2,yis2, theta)

% xis2=robot{robot_num}.packet{i}.hlidar.xs;%-pose(i,1);
% yis2=robot{robot_num}.packet{i}.hlidar.ys;%-pose(i,2);
A= [cosd(theta), sind(theta); -sind(theta), cosd(theta)];

temp_xrob_old= repmat([rob4.pose(ptd(1),1);rob4.pose(ptd(1),2)],1,length(yis2));
temp_xrob_new= repmat([rob4_new.pose(ptd(1),1);rob4_new.pose(ptd(1),2)],1,length(yis2));
tot_is= [xis2,yis2]';
shift_tot= tot_is-temp_xrob_old;
new_is = temp_xrob_new+A*shift_tot;