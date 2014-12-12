robot_num=5; 

%% Init of Map
global MAP; 
%global MAP2; 
MAP.res   = 0.1*1000; %meters

MAP.xmin  = -80*1000;  %meters
MAP.ymin  = -80*1000;
MAP.xmax  =  80*1000;
MAP.ymax  =  80*1000;

% MAP.xmin  = -30*1000;  %meters
% MAP.ymin  = -30*1000;
% MAP.xmax  =  30*1000;
% MAP.ymax  =  30*1000;

% dimensions of the map
MAP.sizex  = ceil((MAP.xmax - MAP.xmin) / MAP.res + 1); %cells
MAP.sizey  = ceil((MAP.ymax - MAP.ymin) / MAP.res + 1);

MAP.map = zeros(MAP.sizex,MAP.sizey,'int8');
MAP2.map = zeros(MAP.sizex,MAP.sizey,'int8');
log_prob= zeros(MAP.sizex,MAP.sizey);
log_odds= zeros(MAP.sizex,MAP.sizey);

%%
c2=clock;
fix(c2)
tic
for i=1:structfun(@numel,robot{1,robot_num})
    i
    %x_rob = pose(i,1);
    %y_rob = pose(i,2);
    
    x_rob = ceil((pose(i,1)*1000 - MAP.xmin) ./ MAP.res);
    y_rob = ceil((pose(i,2)*1000 - MAP.ymin) ./ MAP.res);    
    
    angle = pose(i,4);
    
    plot_x(i)=x_rob;
    plot_y(i)=y_rob;
    
    xis2=robot{robot_num}.packet{i}.hlidar.xs;%-pose(i,1);
    yis2=robot{robot_num}.packet{i}.hlidar.ys;%-pose(i,2);
    
    dist= sqrt(xis2.^2 + yis2.^2);
    
    cat_dist= [xis2, yis2, dist];
    
    %new_cat_dist= cat_dist(cat_dist(:,3)<5, :);
    new_cat_dist=cat_dist(cat_dist(:,3)>0.1, :);
    
    new_cat_dist(:,1)= new_cat_dist(:,1)*1000;%+x_rob;
    new_cat_dist(:,2)= new_cat_dist(:,2)*1000;%+y_rob;
    
    [log_prob, MAP]=updateMap2(log_prob, new_cat_dist(:,1:2),x_rob,y_rob);

end
toc
here= mat2gray(log_prob);
figure, imshow(here); %hold on; plot(-y_robot_new,x_robot_new,'r');
hold on; plot(plot_y(1:i), plot_x(1:i),'r');