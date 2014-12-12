robot_num=4;

for i =1:structfun(@numel,robot{1,robot_num})
    
	xis=robot{robot_num}.packet{i}.hlidar.xs;%-pose(i,1);
	yis=robot{robot_num}.packet{i}.hlidar.ys;%-pose(i,2);
    
    rob4lidar{i}= [xis, yis];
    
    clear xis;
    clear yis
    
end