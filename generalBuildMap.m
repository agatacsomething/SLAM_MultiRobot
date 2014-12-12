load('rob45_probmat.mat');

num_here=0;

order = 2 %[2,3,1];

for popo= 1:length(order)
    
    robot_num=order(popo);   
    rob= load(['robot_' robot_num '.mat']);


    x_rob = ceil((rob.pose(:,1)*1000 - MAP.xmin) ./ MAP.res);
    y_rob = ceil((rob.pose(:,2)*1000 - MAP.ymin) ./ MAP.res); 

    rob_new=rob;

    % figure; plot(y_rob4,x_rob4),
    % hold on; plot(y_rob4(1800:2000),x_rob4(1800:2000),'r')

    global MAP

    angleshift=0;
    %%
    c2=clock;
    fix(c2)
    tic

    num2do = structfun(@numel,robot{1,robot_num})-200;
    
    if i>100
        num_here=num_here+1; 
        absdist(num_here)= sqrt(rob.pose(i,1)-(rob.pose(i-100,1))^2+rob.pose(i,2)-(rob.pose(i-100,2))^2);
    end
    
    i=1;
    remhere=0;
    redo=0;
    while i<num2do
    %i
    i
        %generate map up to point 300
        if 
            x_rob = rob5.pose(i,1);
            y_rob = rob5.pose(i,2);

            xis2=robot{robot_num}.packet{i}.hlidar.xs;%-pose(i,1);
            yis2=robot{robot_num}.packet{i}.hlidar.ys;%-pose(i,2);

            new_cat_dist= [xis2, yis2];

            new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
            new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;

            [log_prob2, MAP]=updateMap2(log_prob2, new_cat_dist,x_rob(i),y_rob(i));

            i=i+1;

        elseif i>219 %&& rem(i,20)==0


            thetas=[-1.5:0.05:1.5] + angleshift;

            [bwwalls] = getWalls(log_prob2);

    %         walls= find(log_prob2>0.8);
    %         bwwalls= zeros(size(log_prob3)); 
    %         bwwalls(walls)= 1; 

            ptd=i:i+99;
            ptd2=i:i+100;

            for j=1:length(thetas)
                [log_prob_temp]= getNewTempMap(ptd,robot,log_prob2,thetas(j), ...
                    robot_num, rob, rob_new);
                bwwalls_temp = getWalls(log_prob_temp);
                corr_angles(j) = corr2(bwwalls,bwwalls_temp);
                %corr_angles(j) = corr2(bwwalls,log_prob_temp);
            end

            [val idx]=max(corr_angles);
            angleshift= thetas(idx) + angleshift;

            if thetas(idx)~=0
                %reset all angles
                %reset robot angles and lidar points between 300 and 320
                [rob4_new]=rotatePartialPose(rob_new, rob, angleshift, ptd2);

                for j=1:length(ptd)

                    xis2=robot{robot_num}.packet{ptd(j)}.hlidar.xs;%-pose(i,1);
                    yis2=robot{robot_num}.packet{ptd(j)}.hlidar.ys;%-pose(i,2);

                    [new_is]= (rotatedLidar(rob, rob_new, ptd,xis2,yis2, angleshift))';
                    new_cat_dist(:,1)= new_is(:,1)*1000;%+x_rob;
                    new_cat_dist(:,2)= new_is(:,2)*1000;%+y_rob;

                    x_rob5_new = ceil((rob_new.pose(ptd(j),1)*1000 - MAP.xmin) ./ MAP.res);
                    y_rob5_new = ceil((rob_new.pose(ptd(j),2)*1000 - MAP.ymin) ./ MAP.res); 

                    [log_prob2, MAP]=updateMap2(log_prob2, new_cat_dist,...
                        x_rob_new,y_rob_new);

                    clear new_cat_dist;
                end

            elseif thetas(idx) ==0

                for j=ptd
                    x_rob = rob.pose(j,1);
                    y_rob = rob.pose(j,2);

                    xis2=robot{robot_num}.packet{j}.hlidar.xs;%-pose(i,1);
                    yis2=robot{robot_num}.packet{j}.hlidar.ys;%-pose(i,2);

                    new_cat_dist= [xis2, yis2];

                    new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
                    new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;

                    [log_prob2, MAP]=updateMap2(log_prob2, new_cat_dist,x_rob(j),y_rob(j));
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
    load chirp
    sound(y,Fs)

    here= mat2gray(log_prob2);
    figure, imshow(here); %hold on; plot(-y_robot_new,x_robot_new,'r');
    %hold on; plot(y_rob5(1:2001), x_rob5(1:2001),'b');
    %hold on; plot(y_rob4_new(1:2069), x_rob4_new(1:2069),'b');

    load chirp
    sound(y,Fs)

end