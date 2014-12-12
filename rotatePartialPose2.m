function [rob4_new]=rotatePartialPose2(rob4_new, rob4, theta, ptd2, xshift, yshift)

A= [cosd(theta), sind(theta), 0, xshift; 
         -sind(theta), cosd(theta), 0, yshift;
         0, 0, 1, 0;
         0, 0, 0, 1];


temp_xrob_old= repmat([rob4.pose(ptd2(1),1);rob4.pose(ptd2(1),2)],1,length(ptd2));
temp_xrob_new= repmat([rob4_new.pose(ptd2(1),1);rob4_new.pose(ptd2(1),2)],1,length(ptd2));

tot_is= [rob4.pose(ptd2,1),rob4.pose(ptd2,2)]';
shift_tot= tot_is-temp_xrob_old;
%shift_tot2= [shift_tot', zeros(length(ptd2)), ones(length(ptd2))];
shift_tot2= [shift_tot; zeros(1,length(ptd2)); ones(1,length(ptd2))];
temp_xrob_new_zeros= [temp_xrob_new;zeros(2,length(ptd2))];
all_pose = temp_xrob_new_zeros+A*shift_tot2;
rob4_new.pose(ptd2,1:2) = (all_pose(1:2,:))';%temp_xrob_new_zeros+A*shift_tot2;

% rob4_new = temp_xrob_new_zeros+A*shift_tot2';
% 
% rob4_new.pose(ptd2,1:2) = (temp_xrob_new+A*shift_tot)';
% 
% 
% A= [cosd(theta), sind(theta); -sind(theta), cosd(theta)];
% temp_xrob_old= repmat([rob4.pose(ptd(1),1);rob4.pose(ptd(1),2)],1,length(yis2));
% temp_xrob_new= repmat([rob4_new.pose(ptd(1),1);rob4_new.pose(ptd(1),2)],1,length(yis2));
% tot_is= [xis2,yis2]';
% shift_tot= tot_is-temp_xrob_old;
% shift_tot2= [shift_tot', zeros(size(yis2)), ones(size(yis2))];
% temp_xrob_new_zeros= [temp_xrob_new;zeros(2,length(yis2))];
% %temp_xrob_old_zeros= [temp_xrob_new,zeros(2,length(yis2))];
% new_is = temp_xrob_new_zeros+A*shift_tot2';


