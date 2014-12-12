function [ocup_points, log_prob, MAP] = getOccPoints(anglespan, lidar_angles, lidar_reading_new, xpos_g, ypos_g,theta_g,log_prob,i)
k=0;
global MAP;

for j = anglespan
	angle = (lidar_angles(j) +theta_g);
	lidar_reading=lidar_reading_new(j,2*i);

	if lidar_reading > .01 && lidar_reading < 5
        k=k+1;

        ocup_points(k,1) = (cos(angle)*(lidar_reading*1000))+xpos_g;
        ocup_points(k,2) = (sin(angle)*(lidar_reading*1000))+ypos_g;
        
        xis(1,1) = ceil((ocup_points(k,1) - MAP.xmin) ./ MAP.res);
        yis(1,1) = ceil((ocup_points(k,2) - MAP.ymin) ./ MAP.res);
                
        indGood = (xis(1,1) > 1) & (yis(1,1) > 1) & (xis(1,1) < MAP.sizex)...
                & (yis(1,1) < MAP.sizey);
    
        if indGood ==1
            inds = sub2ind(size(MAP.map),xis(1,1),yis(1,1));
            log_prob(inds) = log_prob(inds)+0.01; 
            MAP.map(inds) = 100;
        end 

    end
               
          
end