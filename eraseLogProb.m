function [log_prob] = eraseLogProb(x_robot_new_d, y_robot_new_d,...
             anglespan, thetas_d, lidar_reading_new,log_prob,leng, lidar_angles)
global MAP         

    
for i =1:leng
	k=0;
	for j = anglespan
        %angle = thetas_d(i);
        angle = (lidar_angles(j) +thetas_d(i));
        lidar_reading=lidar_reading_new(j,2*i);
        
           
        if lidar_reading_new(j,2*i) > .1 && lidar_reading_new(j,2*i) < 5
            k=k+1;
                
            ocup_points_f(k,1) = (cos(angle)*(lidar_reading*1000))+x_robot_new_d(i);
            ocup_points_f(k,2) = (sin(angle)*(lidar_reading*1000))+y_robot_new_d(i);
              
        end
        
            
               
    end      
        
	x_rob_f = ceil((x_robot_new_d(i) - MAP.xmin) ./ MAP.res);
	y_rob_f = ceil((y_robot_new_d(i) - MAP.ymin) ./ MAP.res);
             
	[log_prob, MAP] = updateMap2(log_prob, ocup_points_f, x_rob_f,y_rob_f);
    clear ocup_points_f;
end
