function [log_prob2] = buildMapGeneral4(num2do,order,log_prob2, robot,yshiftnew,xshiftnew,gogo)

global MAP

for popo= 1:length(order)


    robot_num=order(popo);   
    rob= load(['robot_' num2str(robot_num) '.mat']);

    rob.pose(:,1)=rob.pose(:,1)+xshiftnew; 
    rob.pose(:,2)=rob.pose(:,2)+yshiftnew; 
    
    
    x_rob_x = ceil((rob.pose(:,1)*1000 - MAP.xmin) ./ MAP.res);%+xshift;
    y_rob_x = ceil((rob.pose(:,2)*1000 - MAP.ymin) ./ MAP.res);%+yshift; 

    rob_new=rob;

    tic

    num_here=0;
    go_on=1;

    s2d=50;
    maxlidardist=6.5;
    i=1;
    while i<num2do+1
    %i
    %i


        if rem(i,s2d)==0 && i>s2d
            num_here=num_here+1;
            euc_dist(num_here)= sqrt((rob.pose(i,1)-rob.pose(i-s2d,1))^2+...
                (rob.pose(i,2)-rob.pose(i-s2d,2))^2);


            if euc_dist(num_here)>1 || abs(rob.pose(i,4)-rob.pose(i-s2d,4))>1
                go_on =0;
            elseif euc_dist(num_here)<1
                go_on =1;
            end
           
        end

        %generate map up to point 300
        if go_on==1
            x_rob = rob.pose(i,1);
            y_rob = rob.pose(i,2);

            xis2=robot{robot_num}.packet{i}.hlidar.xs+xshiftnew;%-pose(i,1);
            yis2=robot{robot_num}.packet{i}.hlidar.ys+yshiftnew;%-pose(i,2);

            ydist=[yis2-rob_new.pose(i,2)];
            xdist=[xis2-rob_new.pose(i,1)];
            xdist2=xdist.^2;
            ydist2=ydist.^2;
            newdist=sqrt(ydist2+xdist2);
            tempidx=find(newdist<maxlidardist);
            testx=xis2(tempidx);
            testy=yis2(tempidx);
                    
            %new_cat_dist=[xis2,yis2];
            new_cat_dist= [testx, testy];

            new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
            new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;
            
            xis = ceil((new_cat_dist(:,1) - MAP.xmin) ./ MAP.res);%xshift;
            yis = ceil((new_cat_dist(:,2) - MAP.ymin) ./ MAP.res);%+yshift;


            [log_prob2]=updateMap2(log_prob2, new_cat_dist,x_rob_x(i),y_rob_x(i),xis,yis);

            i=i+1;

        elseif go_on==0 %&& rem(i,20)==0
        i

            thetas=[-5:.1:5];
            shifts= [-1.8:.1:1.8];
            [bwwalls] = getWalls3(log_prob2);

            ptd=i:i+(s2d-1);
            ptd2=i:i+s2d;
            ptd3= i-100:i+99;

            for j=1:length(thetas)
                [log_prob_temp]= getNewTempMap3(ptd3,robot,log_prob2,thetas(j), ...
                    robot_num, rob, rob_new, yshiftnew,xshiftnew, 0,0);
                
                bwwalls_temp = getWalls2(log_prob_temp);
                
                corr_angles(j) = corr2(bwwalls,bwwalls_temp);
            end
            
            [val idx]=max(corr_angles);
            angleshift= thetas(idx);

            if gogo==1
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
                %xshift=0;
                log_prob2_thx= getNewTempMap2(ptd3,robot,log_prob2,angleshift, ...
                    robot_num, rob, rob_new, yshiftnew,xshift,0);
                bwwalls_thx= getWalls3(log_prob2_thx);

                for m=1:length(shifts)
                    [log_prob_temp_y]= getNewTempMap2(ptd3,robot,log_prob2,angleshift, ...
                        robot_num, rob, rob_new, yshiftnew,xshift, shifts(m));

                    bwwalls_temp_y = getWalls2(log_prob_temp_y);

                    corr_angles_y(m) = corr2(bwwalls_thx,bwwalls_temp_y);

                end

                [val_y, idx_y]=max(corr_angles_y);
                yshift= shifts(idx_y);
            else
                yshift=0;
                xshift=0;
            end
            %toc
            
            if angleshift~=0 || xshift~=0 || yshift ~=0
                %reset all angles
                %reset robot angles and lidar points between 300 and 320
                [rob_new]=rotatePartialPose2(rob_new, rob, angleshift, ptd2, xshift, yshift);

                for j=1:length(ptd)

                    xis2=robot{robot_num}.packet{ptd(j)}.hlidar.xs+xshiftnew;%-pose(i,1);
                    yis2=robot{robot_num}.packet{ptd(j)}.hlidar.ys+yshiftnew;%-pose(i,2);

                    ydist=[yis2-rob_new.pose(ptd(j),2)];
                    xdist=[xis2-rob_new.pose(ptd(j),1)];
                    xdist2=xdist.^2;
                    ydist2=ydist.^2;
                    newdist=sqrt(ydist2+xdist2);
                    tempidx=find(newdist<maxlidardist);
                    testx=xis2(tempidx);
                    testy=yis2(tempidx);
                    
                    %[new_is]= (rotatedLidar(rob, rob_new, ptd,xis2,yis2, angleshift))';
                    %[new_is]= (rotatedLidar2(rob, rob_new, ptd,xis2,yis2, angleshift,xshift,yshift))';
                    [new_is]= (rotatedLidar2(rob, rob_new, ptd,testx,testy, angleshift,xshift,yshift))';
                    
                    new_cat_dist(:,1)= new_is(:,1)*1000;%+x_rob;
                    new_cat_dist(:,2)= new_is(:,2)*1000;%+y_rob;

                    xis = ceil((new_cat_dist(:,1) - MAP.xmin) ./ MAP.res);%+xshift;
                    yis = ceil((new_cat_dist(:,2) - MAP.ymin) ./ MAP.res);%+yshift;
                    
                    x_rob_new = ceil((rob_new.pose(ptd(j),1)*1000 - MAP.xmin) ./ MAP.res);%+xshift;
                    y_rob_new = ceil((rob_new.pose(ptd(j),2)*1000 - MAP.ymin) ./ MAP.res);%+yshift; 

                    [log_prob2]=updateMap2(log_prob2, new_cat_dist,...
                        x_rob_new,y_rob_new,xis,yis);

                    clear new_cat_dist;
                    clear xis2;
                    clear yis2;
                    clear ydist;
                    clear xdist; 
                    clear newdist; 
                    clear tempidx; 
                    clear testy; 
                    clear testx;
                end

            else %if thetas(idx) ==0 

                for j=length(ptd)
                    x_rob = rob.pose(ptd(j),1);
                    y_rob = rob.pose(ptd(j),2);

                    xis2=robot{robot_num}.packet{j}.hlidar.xs+xshiftnew;%-pose(i,1);
                    yis2=robot{robot_num}.packet{j}.hlidar.ys+yshiftnew;%-pose(i,2);

                    ydist=[yis2-rob_new.pose(ptd(j),2)];
                    xdist=[xis2-rob_new.pose(ptd(j),1)];
                    xdist2=xdist.^2;
                    ydist2=ydist.^2;
                    newdist=sqrt(ydist2+xdist2);
                    tempidx=find(newdist<maxlidardist);
                    testx=xis2(tempidx);
                    testy=yis2(tempidx);
                    
                    new_cat_dist= [xis2, yis2];

                    new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
                    new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;
                    
                    xis = ceil((new_cat_dist(:,1) - MAP.xmin) ./ MAP.res);%+xshift;
                    yis = ceil((new_cat_dist(:,2) - MAP.ymin) ./ MAP.res);%+yshift;

                    [log_prob2]=updateMap2(log_prob2, new_cat_dist,x_rob_x(j),y_rob_x(j),xis,yis);
                    clear new_cat_dist;
                    clear xis2;
                    clear yis2;
                    clear ydist;
                    clear xdist; 
                    clear newdist; 
                    clear tempidx; 
                    clear testy; 
                    clear testx;
                end
            end

            i=i+s2d;
        end

        %[log_prob2, MAP]=updateMap2(log_prob2, new_cat_dist,x_rob4(i),y_rob4(i));
        %i=i+1;
    clear new_cat_dist;
    clear xis2
    clear yis2
    clear ydist;
    clear xdist;
    clear newdist;
    clear tempidx;
    clear testy;
    clear testx;
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
