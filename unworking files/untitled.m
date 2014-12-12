res2   = 0.1*1000; %meters

xmin2  = -25*1000;  %meters
ymin2  = -5*1000;
xmax2  =  25*1000;
ymax2  =  25*1000;


% dimensions of the map
sizex2  = ceil((xmax2 - xmin2) / res2 + 1) %cells
sizey2  = ceil((ymax2 - ymin2) / res2 + 1)