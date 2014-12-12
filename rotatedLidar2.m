function [new_is]= rotatedLidar2(rob4, rob4_new, ptd,xis2,yis2, theta, xshift, yshift)

A= [cosd(theta), sind(theta), 0, xshift; 
         -sind(theta), cosd(theta), 0, yshift;
         0, 0, 1, 0;
         0, 0, 0, 1];

temp_xrob_old= repmat([rob4.pose(ptd(1),1);rob4.pose(ptd(1),2)],1,length(yis2));
temp_xrob_new= repmat([rob4_new.pose(ptd(1),1);rob4_new.pose(ptd(1),2)],1,length(yis2));
tot_is= [xis2,yis2]';
shift_tot= tot_is-temp_xrob_old;
shift_tot2= [shift_tot', zeros(size(yis2)), ones(size(yis2))];
temp_xrob_new_zeros= [temp_xrob_new;zeros(2,length(yis2))];
%temp_xrob_old_zeros= [temp_xrob_new,zeros(2,length(yis2))];
new_is = temp_xrob_new_zeros+A*shift_tot2';



