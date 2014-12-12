%x_rob4 = ceil((rob4.pose(450:end,1)*1000 - MAP.xmin) ./ MAP.res);
%y_rob4 = ceil((rob4.pose(450:end,2)*1000 - MAP.ymin) ./ MAP.res); 

%function getNewTempMap(ptd,robot,

clear log_prob_temp;

ptd=660:700;

% xpoints= x_rob4(ptd)
% ypoints= y_rob4(ptd)    

% here= mat2gray(log_prob2);
% figure, imshow(here);
% hold on; plot(ypoints,xpoints,'y')
theta=30;

log_prob_temp=zeros(size(log_prob2)); 

for i=ptd
     A= [cosd(theta), sind(theta); -sind(theta), cosd(theta)];
    
	xis2=robot{robot_num}.packet{i}.hlidar.xs;%-pose(i,1);
    yis2=robot{robot_num}.packet{i}.hlidar.ys;%-pose(i,2);
    
    %xis = ceil((xis2(:,1)*1000 - MAP.xmin) ./ MAP.res);
    %yis = ceil((yis2(:,1)*1000 - MAP.ymin) ./ MAP.res);

    temp_xrob= repmat([rob4.pose(i,1);rob4.pose(i,1)],1,length(yis2));
    tot_is= [xis2,yis2]';
    shift_tot= tot_is-temp_xrob;
    new_is = temp_xrob+A*shift_tot;
    
	xis(1,:) = ceil((new_is(1,:)*1000 - MAP.xmin) ./ MAP.res);
	yis(1,:) = ceil((new_is(2,:)*1000 - MAP.ymin) ./ MAP.res);

    for k = 1:length(xis)
        indGood = (xis(1,k) > 1) & (yis(1,k) > 1) & (xis(1,k) < MAP.sizex)...
            & (yis(1,k) < MAP.sizey);
        
        if indGood ==1
            inds = sub2ind(size(MAP.map),xis(1,k),yis(1,k));
            log_prob_temp(inds) = 1; 
                %log_prob_walls(inds)= 1;
        end
    end
clear xis2
clear yis2
clear xis
clear yis
    
end


here2= mat2gray(log_prob_temp);
figure, imshow(here2); %hold on; plot(-y_robot_new,x_robot_new,'r');
%hold on; plot(ypoints, xpoints,'r');