function [ocup_points] = regenOccPoints(lidar_angles,i,theta_f, anglespan, lidar_reading_new, xpos_g, ypos_g)
k=0;
for j = anglespan
	angle = (lidar_angles(j) +theta_f);
	lidar_reading=lidar_reading_new(j,2*i);
        
	if lidar_reading_new(j,2*i) > .1 && lidar_reading_new(j,2*i) < 4
        k=k+1;

        ocup_points(k,1) = (cos(angle)*(lidar_reading*1000))+xpos_g;
        ocup_points(k,2) = (sin(angle)*(lidar_reading*1000))+ypos_g;
    
	end
               
end  


    