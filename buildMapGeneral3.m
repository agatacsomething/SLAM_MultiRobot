function [log_prob2] = buildMapGeneral3(num2do,order,log_prob2, robot,yshiftnew)

%load('rob45_probmat.mat');
    global MAP
num_here=0;

%order = 2 %['2','3','1'];

for popo= 1:length(order)


    robot_num=order(popo);   
    rob= load(['robot_' num2str(robot_num) '.mat']);

    rob.pose(:,2)=rob.pose(:,2)+yshiftnew; 
    
    x_rob_x = ceil((rob.pose(:,1)*1000 - MAP.xmin) ./ MAP.res);%+xshift;
    y_rob_x = ceil((rob.pose(:,2)*1000 - MAP.ymin) ./ MAP.res);%+yshift; 

    rob_new=rob;

    %num2do = structfun(@numel,robot{1,robot_num})-200;
    % figure; plot(y_rob4,x_rob4),
    % hold on; plot(y_rob4(1800:2000),x_rob4(1800:2000),'r')



    angleshift=0;
    %%
%     c2=clock;
%     fix(c2)
    tic
    part2do=1:753;
    %for i=part2do %1:structfun(@numel,robot{1,robot_num})
    i=1; 
    remhere=0;
    num_here=0;
    go_on=1;
    redo=0;
    while i<num2do+1
    %i
    %i


        if rem(i,20)==0 && i>20
            num_here=num_here+1;
            euc_dist(num_here)= sqrt((rob.pose(i,1)-rob.pose(i-20,1))^2+...
                (rob.pose(i,2)-rob.pose(i-20,2))^2);

            if euc_dist(num_here)<1
                go_on =1;
            else
                go_on =0;
            end
            
            %saved_i( = saved_num+1;
        end


        %generate map up to point 300
        if go_on==1
            x_rob = rob.pose(i,1);
            y_rob = rob.pose(i,2);

            xis2=robot{robot_num}.packet{i}.hlidar.xs;%-pose(i,1);
            yis2=robot{robot_num}.packet{i}.hlidar.ys+yshiftnew;%-pose(i,2);

            new_cat_dist= [xis2, yis2];

            new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
            new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;
            
            xis = ceil((new_cat_dist(:,1) - MAP.xmin) ./ MAP.res);%xshift;
            yis = ceil((new_cat_dist(:,2) - MAP.ymin) ./ MAP.res);%+yshift;


            [log_prob2]=updateMap2(log_prob2, new_cat_dist,x_rob_x(i),y_rob_x(i),xis,yis);

            i=i+1;

        elseif go_on==0 %&& rem(i,20)==0
        i
    %         if angleshift==0
    %             thetas=[-0.2:0.01:0.2];
    %         else
    %             thetas=[-0.2:0.01:0.2]*angleshift + angleshift;
    %         end
            %thetas=[1.2:0.05:3];
            thetas=[-3:.05:3];
            shifts= [-1.8:.1:1.8];
            [bwwalls] = getWalls3(log_prob2);
            %bwwalls_noise = imdilate(bwwalls,ones(3));
    %         walls= find(log_prob2>0.8);
    %         bwwalls= zeros(size(log_prob3)); 
    %         bwwalls(walls)= 1; 

            ptd=i:i+19;
            ptd2=i:i+20;
            ptd3= i-20:i+20;

            %tic
            %here=1
            for j=1:length(thetas)
                [log_prob_temp]= getNewTempMap2(ptd3,robot,log_prob2,thetas(j), ...
                    robot_num, rob, rob_new, yshiftnew,0,0);
                
                bwwalls_temp = getWalls2(log_prob_temp);
                
                corr_angles(j) = corr2(bwwalls,bwwalls_temp);
                
            end
            
            [val idx]=max(corr_angles);
            angleshift= thetas(idx);
       %xshift=0;
            
            log_prob2_th= getNewTempMap2(ptd3,robot,log_prob2,angleshift, ...
                    robot_num, rob, rob_new, yshiftnew,0,0);
            bwwalls_th= getWalls3(log_prob2_th);

            for k=1:length(shifts)
                [log_prob_temp_x]= getNewTempMap2(ptd3,robot,log_prob2,angleshift, ...
                    robot_num, rob, rob_new, yshiftnew,shifts(k),0);
                
                bwwalls_temp_x = getWalls2(log_prob_temp_x);
                
                corr_angles_x(k) = corr2(bwwalls_th,bwwalls_temp_x);
                
            end

            
            [val_x, idx_x]=max(corr_angles_x);
            xshift= shifts(idx_x);
            
%             log_prob2_thx= getNewTempMap2(ptd3,robot,log_prob2,angleshift, ...
%                 robot_num, rob, rob_new, yshiftnew,xshift,0);
%             bwwalls_thx= getWalls3(log_prob2_thx);
%             
%             for m=1:length(shifts)
%                 [log_prob_temp_y]= getNewTempMap2(ptd3,robot,log_prob2,angleshift, ...
%                     robot_num, rob, rob_new, yshiftnew,xshift, shifts(k));
%                 
%                 bwwalls_temp_y = getWalls2(log_prob_temp_y);
%                 
%                 corr_angles_y(k) = corr2(bwwalls_thx,bwwalls_temp_y);
%                 
%             end
% 
%             [val_y, idx_y]=max(corr_angles_y);
            yshift= 0; %shifts(idx_y);
            %toc
            
            if thetas(idx)~=0 || xshift~=0 || yshift ~=0
                %reset all angles
                %reset robot angles and lidar points between 300 and 320
                [rob_new]=rotatePartialPose2(rob_new, rob, angleshift, ptd2, xshift, yshift);

                for j=1:length(ptd)

                    xis2=robot{robot_num}.packet{ptd(j)}.hlidar.xs;%-pose(i,1);
                    yis2=robot{robot_num}.packet{ptd(j)}.hlidar.ys+yshiftnew;%-pose(i,2);

                    %[new_is]= (rotatedLidar(rob, rob_new, ptd,xis2,yis2, angleshift))';
                    [new_is]= (rotatedLidar2(rob, rob_new, ptd,xis2,yis2, angleshift,xshift,yshift))';
                    
                    new_cat_dist(:,1)= new_is(:,1)*1000;%+x_rob;
                    new_cat_dist(:,2)= new_is(:,2)*1000;%+y_rob;

                    xis = ceil((new_cat_dist(:,1) - MAP.xmin) ./ MAP.res);%+xshift;
                    yis = ceil((new_cat_dist(:,2) - MAP.ymin) ./ MAP.res);%+yshift;
                    
                    x_rob_new = ceil((rob_new.pose(ptd(j),1)*1000 - MAP.xmin) ./ MAP.res);%+xshift;
                    y_rob_new = ceil((rob_new.pose(ptd(j),2)*1000 - MAP.ymin) ./ MAP.res);%+yshift; 

                    [log_prob2]=updateMap2(log_prob2, new_cat_dist,...
                        x_rob_new,y_rob_new,xis,yis);

                    clear new_cat_dist;
                end

            else %if thetas(idx) ==0 

                for j=ptd
                    x_rob = rob.pose(j,1);
                    y_rob = rob.pose(j,2);

                    xis2=robot{robot_num}.packet{j}.hlidar.xs;%-pose(i,1);
                    yis2=robot{robot_num}.packet{j}.hlidar.ys+yshiftnew;%-pose(i,2);

                    new_cat_dist= [xis2, yis2];

                    new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
                    new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;
                    
                    xis = ceil((new_cat_dist(:,1) - MAP.xmin) ./ MAP.res);%+xshift;
                    yis = ceil((new_cat_dist(:,2) - MAP.ymin) ./ MAP.res);%+yshift;

                    [log_prob2]=updateMap2(log_prob2, new_cat_dist,x_rob_x(j),y_rob_x(j),xis,yis);
                    clear new_cat_dist;
                end
            end

            i=i+20;
        end

        %[log_prob2, MAP]=updateMap2(log_prob2, new_cat_dist,x_rob4(i),y_rob4(i));
        %i=i+1;
    clear new_cat_dist;
    clear xis2
    clear yis2
    end
end
toc
% here= mat2gray(log_prob3);
% figure, imshow(here); %hold on; plot(-y_robot_new,x_robot_new,'r');
% hold on; plot(y_rob4(part2do), x_rob4(part2do),'r');
% load chirp
% sound(y,Fs)
% 
% here= mat2gray(log_prob2);
% figure, imshow(here); %hold on; plot(-y_robot_new,x_robot_new,'r');
% hold on; plot(y_rob5(1:2001), x_rob5(1:2001),'b');
% %hold on; plot(y_rob4_new(1:2069), x_rob4_new(1:2069),'b');
% 
% load chirp
% sound(y,Fs)

end
