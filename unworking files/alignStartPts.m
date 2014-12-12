 
for robot_num=1:5
    %robot_num=order(popo);   
	rob= load(['robot_' num2str(robot_num) '.mat']);

    x_rob_x(robot_num) = ceil((rob.pose(1,1)*1000 - MAP.xmin) ./ MAP.res);%+xshift;
    y_rob_x(robot_num) = ceil((rob.pose(1,2)*1000 - MAP.ymin) ./ MAP.res);%+yshift;
end