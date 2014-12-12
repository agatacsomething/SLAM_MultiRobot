initvarsometest2

clear log_prob2;
x_rob4 = ceil((rob4.pose(:,1)*1000 - MAP.xmin) ./ MAP.res);
y_rob4 = ceil((rob4.pose(:,2)*1000 - MAP.ymin) ./ MAP.res); 

rob4_new=rob2;

x_im = MAP.xmin:MAP.res:MAP.xmax; %x-positions of each pixel of the map
y_im = MAP.ymin:MAP.res:MAP.ymax; %y-positions of each pixel of the map

x_range = -100:5:100;
y_range = -100:5:100;

% figure; plot(y_rob4,x_rob4),
% hold on; plot(y_rob4(1800:2000),x_rob4(1800:2000),'r')

global MAP
robot_num=4; 
log_prob2= zeros(MAP.sizex,MAP.sizey);
log_prob3= zeros(MAP.sizex,MAP.sizey);
log_prob_walls=log_prob2;
oldtheta=0;
lasttheta=oldtheta;
angleshift=0;
%%
c2=clock;
fix(c2)
tic
part2do=1:753;
%for i=part2do %1:structfun(@numel,robot{1,robot_num})
i=1;
remhere=0;
redo=0;
while i<501
%i
i
    %generate map up to point 300
    if i<1520
        x_rob = rob4.pose(i,1);
        y_rob = rob4.pose(i,2);
    
        xis2=robot{robot_num}.packet{i}.hlidar.xs;%-pose(i,1);
        yis2=robot{robot_num}.packet{i}.hlidar.ys;%-pose(i,2);
        
        new_cat_dist= [xis2, yis2];

        new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
        new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;
        
        [log_prob2, MAP]=updateMap2(log_prob2, new_cat_dist,x_rob4(i),y_rob4(i));

        i=i+1;
    
    elseif i>1519 %&& rem(i,20)==0
        
        
        thetas=[-1.5:0.05:1.5] + angleshift;
        
        [bwwalls] = getWalls(log_prob2);
        
%         walls= find(log_prob2>0.8);
%         bwwalls= zeros(size(log_prob3)); 
%         bwwalls(walls)= 1; 
        
        ptd=i:i+99;
        ptd2=i:i+100;
        
        for j=1:length(thetas)
            [log_prob_temp]= getNewTempMap(ptd,robot,log_prob2,thetas(j), ...
                robot_num, rob4, rob4_new);
            bwwalls_temp = getWalls(log_prob_temp);
            corr_angles(j) = corr2(bwwalls,bwwalls_temp);
            %corr_angles(j) = corr2(bwwalls,log_prob_temp);
        end
        
        [val idx]=max(corr_angles);
        angleshift= thetas(idx) + angleshift;
        
        if thetas(idx)~=0
            %reset all angles
            %reset robot angles and lidar points between 300 and 320
            [rob4_new]=rotatePartialPose(rob4_new, rob4, angleshift, ptd2);
        
            for j=1:length(ptd)

                xis2=robot{robot_num}.packet{ptd(j)}.hlidar.xs;%-pose(i,1);
                yis2=robot{robot_num}.packet{ptd(j)}.hlidar.ys;%-pose(i,2);
                
                [new_is]= (rotatedLidar(rob4, rob4_new, ptd,xis2,yis2, angleshift))';
                new_cat_dist(:,1)= new_is(:,1)*1000;%+x_rob;
                new_cat_dist(:,2)= new_is(:,2)*1000;%+y_rob;

                x_rob4_new = ceil((rob4_new.pose(ptd(j),1)*1000 - MAP.xmin) ./ MAP.res);
                y_rob4_new = ceil((rob4_new.pose(ptd(j),2)*1000 - MAP.ymin) ./ MAP.res); 

                [log_prob2, MAP]=updateMap2(log_prob2, new_cat_dist,...
                    x_rob4_new,y_rob4_new);
                
                clear new_cat_dist;
            end
            
        elseif thetas(idx) ==0
            
            for j=ptd
                x_rob = rob4.pose(j,1);
                y_rob = rob4.pose(j,2);

                xis2=robot{robot_num}.packet{j}.hlidar.xs;%-pose(i,1);
                yis2=robot{robot_num}.packet{j}.hlidar.ys;%-pose(i,2);

                new_cat_dist= [xis2, yis2];

                new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
                new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;

                [log_prob2, MAP]=updateMap2(log_prob2, new_cat_dist,x_rob4(j),y_rob4(j));
                clear new_cat_dist;
            end
        end
        
        i=i+100;
    end
    
	%[log_prob2, MAP]=updateMap2(log_prob2, new_cat_dist,x_rob4(i),y_rob4(i));
	%i=i+1;
clear new_cat_dist;
clear xis2
clear yis2
end
toc
% here= mat2gray(log_prob3);
% figure, imshow(here); %hold on; plot(-y_robot_new,x_robot_new,'r');
% hold on; plot(y_rob4(part2do), x_rob4(part2do),'r');

here= mat2gray(log_prob2);
figure, imshow(here); %hold on; plot(-y_robot_new,x_robot_new,'r');
hold on; plot(y_rob4(1:2001), x_rob4(1:2001),'r');
%hold on; plot(y_rob4_new(1:2069), x_rob4_new(1:2069),'b');

load chirp
sound(y,Fs)

% here2= mat2gray(log_prob_walls+0.5);
% figure, imshow(here2); %hold on; plot(-y_robot_new,x_robot_new,'r');
% hold on; plot(y_rob4(part2do), x_rob4(part2do),'r');





% 
%     if angleshift ==0
%         x_rob = rob4.pose(i,1);
%         y_rob = rob4.pose(i,2);
%     
%         xis2=robot{robot_num}.packet{i}.hlidar.xs;%-pose(i,1);
%         yis2=robot{robot_num}.packet{i}.hlidar.ys;%-pose(i,2);
%     
%         %dist= sqrt(xis2.^2 + yis2.^2);
% 
%         new_cat_dist= [xis2, yis2];
% 
%         %new_cat_dist= cat_dist(cat_dist(:,3)<5, :);
%         %new_cat_dist=cat_dist(cat_dist(:,3)>0.1, :);
% 
%         new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
%         new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;
%     else
%         A= [cosd(angleshift), sind(angleshift); -sind(angleshift), cosd(angleshift)];
%     
%         xis2=robot{robot_num}.packet{i}.hlidar.xs;%-pose(i,1);
%         yis2=robot{robot_num}.packet{i}.hlidar.ys;%-pose(i,2);
%         
%         temp_xrob= repmat([rob4.pose(i,1);rob4.pose(i,2)],1,length(yis2));
%         tot_is= [xis2,yis2]';
%         shift_tot= tot_is-temp_xrob;
%         new_is = temp_xrob+A*shift_tot;
% 
%         new_cat_dist= ([new_is(1,:); new_is(2,:)]*1000)';
%     end