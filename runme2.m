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
rob4= load('robot_4.mat');
rob5= load('robot_5.mat');

log_prob = getProbMat(rob4.pose, 4, robot, log_prob);
log_prob2 = getProbMat(rob5.pose, 5, robot, log_prob);

x_rob4 = ceil((rob4.pose(:,1)*1000 - MAP.xmin) ./ MAP.res);
y_rob4 = ceil((rob4.pose(:,2)*1000 - MAP.ymin) ./ MAP.res); 
    
x_rob5 = ceil((rob5.pose(:,1)*1000 - MAP.xmin) ./ MAP.res);
y_rob5 = ceil((rob5.pose(:,2)*1000 - MAP.ymin) ./ MAP.res);       

here= mat2gray(log_prob2);
figure, imshow(here); %hold on; plot(-y_robot_new,x_robot_new,'r');
hold on; plot(y_rob4, x_rob4,'r');
hold on; plot(y_rob5, x_rob5,'b');
