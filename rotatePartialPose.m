function [rob4_new]=rotatePartialPose(rob4_new, rob4, theta, ptd2)


A= [cosd(theta), sind(theta); -sind(theta), cosd(theta)];
temp_xrob_old= repmat([rob4.pose(ptd2(1),1);rob4.pose(ptd2(1),2)],1,length(ptd2));
temp_xrob_new= repmat([rob4_new.pose(ptd2(1),1);rob4_new.pose(ptd2(1),2)],1,length(ptd2));

tot_is= [rob4.pose(ptd2,1),rob4.pose(ptd2,2)]';
shift_tot= tot_is-temp_xrob_old;
rob4_new.pose(ptd2,1:2) = (temp_xrob_new+A*shift_tot)';


