% 
% 
% for i= 1: length(inds)
%     MAP.map(inds) = MAP.map(inds) +10;
% end
clear MAP
MAP.res   = 0.1*1000; %meters

MAP.xmin  = -25*1000;  %meters
MAP.ymin  = -25*1000;
MAP.xmax  =  25*1000;
MAP.ymax  =  25*1000;



% dimensions of the map
MAP.sizex  = ceil((MAP.xmax - MAP.xmin) / MAP.res + 1); %cells
MAP.sizey  = ceil((MAP.ymax - MAP.ymin) / MAP.res + 1);

MAP.map = zeros(MAP.sizex,MAP.sizey,'int8');


xis = ceil((ocup_points(:,1) - MAP.xmin) ./ MAP.res);
yis = ceil((ocup_points(:,2) - MAP.ymin) ./ MAP.res);

[testa testb] =histc(inds,unique(inds));
inds = sub2ind(size(MAP.map),xis(indGood),yis(indGood));

k=length(testa); 
newinds = unique(inds); 
testa = testa>50; 

MAP.map(newinds) = testa*100;

figure('name','Filled Map 2');
imagesc(MAP.map);