num_here=1;
rob= load('robot_2.mat');
vals_coor(num_here,1)= rob.pose(1,1);
vals_coor(num_here,2)= rob.pose(1,2);

x_rob_x = ceil((rob.pose(:,1)*1000 - MAP.xmin) ./ MAP.res);%+xshift;
y_rob_x = ceil((rob.pose(:,2)*1000 - MAP.ymin) ./ MAP.res);%+yshift; 

for j=101:1700
if rem(j,10)==0

num_here=num_here+1;


euc_dist(num_here-1,1)=(num_here-1)*10;
euc_dist(num_here-1,2)= sqrt((rob.pose(j,1)-rob.pose(j-10,1))^2+(rob.pose(j,2)-rob.pose(j-10,2))^2);


           if euc_dist(num_here-1)<2
                go_on =1;
            else
                go_on =0;
            end
end
end


euc_dist