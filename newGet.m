% clear non_occ_points; 
% %[123, 159]
% %[100, 150]
% x_rob = 100;
% y_rob =150; 
% xis2 = 99; 
% yis2 = 137; 
function [non_occ_points] = newGet(x_rob, y_rob, xis2, yis2)

x_diff_cur = x_rob-xis2;
y_diff_cur = y_rob-yis2;

i=1; 

if x_diff_cur>0 && y_diff_cur>0
    non_occ_points(i,1) = y_rob-1; 
    non_occ_points(i,2) = x_rob-1;
    x_ch =1; 
elseif x_diff_cur<0 && y_diff_cur>0
    non_occ_points(i,1) = y_rob-1;
    non_occ_points(i,2) = x_rob+1;
    x_ch =1; 
elseif x_diff_cur>0 && y_diff_cur<0
    non_occ_points(i,1) = y_rob+1; 
    non_occ_points(i,2) = x_rob-1;
    x_ch =1; 
elseif x_diff_cur<0 && y_diff_cur<0
    non_occ_points(i,1) = y_rob+1;
    non_occ_points(i,2) = x_rob+1;
    x_ch =0; 
elseif x_diff_cur==0 && y_diff_cur>0
    non_occ_points(i,1) = y_rob-1;
    non_occ_points(i,2) = x_rob;
    x_ch =0;
elseif x_diff_cur==0 && y_diff_cur<0
    non_occ_points(i,1) = y_rob+1;
    non_occ_points(i,2) = x_rob;
    x_ch =0;
elseif y_diff_cur==0 && x_diff_cur>0
    non_occ_points(i,1) = y_rob;
    non_occ_points(i,2) = x_rob-1;
    x_ch =1;
else
    non_occ_points(i,1) = y_rob;
    non_occ_points(i,2) = x_rob+1;
    x_ch =1;
end

cur_endy = non_occ_points(i,1);
cur_endx = non_occ_points(i,2);

x_diff_cur = non_occ_points(i,2)-xis2;
y_diff_cur = non_occ_points(i,1)-xis2;

% x_diff_old = x_diff_cur;
% y_diff_old = y_diff_cur;
% 
% x_diff_cur= ;
% y_diff_cur

while ((cur_endy ~=yis2)+(cur_endx ~=xis2))~=0
    if ((cur_endy ~=yis2)+(cur_endx ~=xis2))==0
        break
    end
    
    i=i+1;
    
    if x_diff_cur>0 && y_diff_cur>0 %&& x_ch ==0
        non_occ_points(i,1) = non_occ_points(i-1,1); 
        non_occ_points(i,2) = non_occ_points(i-1,2)-1;
        x_ch =1; 
    elseif x_diff_cur<0 && y_diff_cur>0
        non_occ_points(i,1) = non_occ_points(i-1,1)-1;
        non_occ_points(i,2) = non_occ_points(i-1,2)+1;
        x_ch =1; 
    elseif x_diff_cur>0 && y_diff_cur<0
        non_occ_points(i,1) = non_occ_points(i-1,1)+1; 
        non_occ_points(i,2) = non_occ_points(i-1,2)-1;
        x_ch =1; 
    elseif x_diff_cur<0 && y_diff_cur<0
        non_occ_points(i,1) = non_occ_points(i-1,1)+1;
        non_occ_points(i,2) = non_occ_points(i-1,2)+1;
        x_ch =0; 
    elseif x_diff_cur==0 && y_diff_cur>0
        non_occ_points(i,1) = non_occ_points(i-1,1)-1;
        non_occ_points(i,2) = non_occ_points(i-1,2);
        x_ch =0;
    elseif x_diff_cur==0 && y_diff_cur<0
        non_occ_points(i,1) = non_occ_points(i-1,1)+1;
        non_occ_points(i,2) = non_occ_points(i-1,2);
        x_ch =0;
    elseif y_diff_cur==0 && x_diff_cur>0
        non_occ_points(i,1) = non_occ_points(i-1,1)-1;
        non_occ_points(i,2) = non_occ_points(i-1,2);
        x_ch =1;
    else
        non_occ_points(i,1) = non_occ_points(i-1,1);
        non_occ_points(i,2) = non_occ_points(i-1,2)+1;
        x_ch =1;
    end

    
    cur_endy = non_occ_points(i,1);
    cur_endx = non_occ_points(i,2);
    
x_diff_cur = non_occ_points(i,2)-xis2;
y_diff_cur = non_occ_points(i,1)-yis2;
end

%non_occ_points=non_occ_points(2:end-1,:);
% ys= non_occ_points(:,1);
% xs= non_occ_points(:,2);
