k=1; 
for j = anglespan
                angle = (lidar_angles(j) +thetas_d(i-1));
                lidar_reading=lidar_reading_new(j,2*(i-1));
        
            if lidar_reading_new(j,2*(i-1)) > .1 && lidar_reading_new(j,2*(i-1)) < 4
                k=k+1;
                
                ocup_points(k,1) = (cos(angle)*(lidar_reading*1000))+xpos_g(o);
                ocup_points(k,2) = (sin(angle)*(lidar_reading*1000))+ypos_g(o);
                ocup_points(k,3) = angle;

                xis(k,1) = ceil((ocup_points(k,2*(o-1)+1) - MAP.xmin) ./ MAP.res);
                yis(k,1) = ceil((ocup_points(k,2*(o-1)+2) - MAP.ymin) ./ MAP.res);
              
            end
               
end      