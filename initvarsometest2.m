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
blank_log_prob= zeros(MAP.sizex,MAP.sizey);
log_odds= zeros(MAP.sizex,MAP.sizey);

%%
%rob4= load('robot_2.mat');
%rob5= load('robot_5.mat');