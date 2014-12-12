%function [non_occ_points] = getNonOccPoints(x_rob, y_rob, xis, yis)
clear non_occ_points
i=1;
cur_slope = (yis-y_rob)/(xis-x_rob); 
is_cs_inf = isinf(cur_slope); 

if cur_slope>1
    non_occ_points(i,1) = yis; 
    non_occ_points(i,2) = xis+1; 
    x_ch =1; 
elseif cur_slope<1
    non_occ_points(i,1) = yis;
    non_occ_points(i,2) = xis-1;
    x_ch =1; 
elseif is_cs_inf==1
    non_occ_points(i,1) = yis+1;
    non_occ_points(i,2) = xis;
    x_ch =0;
end

cur_slope = (non_occ_points(i,1)-y_rob)/(non_occ_points(i,2)-x_rob); 
is_cs_inf = isinf(cur_slope); 

cur_endy = non_occ_points(i,1);
cur_endx = non_occ_points(i,2);



while ((cur_endy ~=y_rob)+(cur_endx ~=x_rob))~=0
    i=i+1
    
    if (cur_slope>1 && x_ch ==0) && is_cs_inf==0 && cur_slope~=0
        non_occ_points(i,1) = non_occ_points(i-1,1); 
        non_occ_points(i,2) = non_occ_points(i-1,2)+1; 
        x_ch =1; 
    elseif (cur_slope==1)&& is_cs_inf==0&& cur_slope~=0
        non_occ_points(i,1) = non_occ_points(i-1,1); 
        non_occ_points(i,2) = non_occ_points(i-1,2)-1; 
        x_ch =1; 
    elseif (cur_slope>1 && x_ch ==1)&& is_cs_inf==0&& cur_slope~=0
        non_occ_points(i,1) = non_occ_points(i-1,1)-1; 
        non_occ_points(i,2) = non_occ_points(i-1,2); 
        y_ch =1; 
    elseif (cur_slope<1 && x_ch ==0)&& is_cs_inf==0&& cur_slope~=0
        non_occ_points(i,1) = non_occ_points(i-1,1);
        non_occ_points(i,2) = non_occ_points(i-1,2)-1;
        x_ch = 1;
	elseif (cur_slope<1 && x_ch ==1)&& is_cs_inf==0&& cur_slope~=0
        non_occ_points(i,1) = non_occ_points(i-1,1)+1; 
        non_occ_points(i,2) = non_occ_points(i-1,2); 
        x_ch =0;
    elseif cur_slope==0
        %here=i
        non_occ_points(i,1) = non_occ_points(i-1,1);
        non_occ_points(i,2) = non_occ_points(i-1,2)-1;
        x_ch =1;     
    elseif is_cs_inf==1
        %here=i
        non_occ_points(i,1) = non_occ_points(i-1,1)+1;
        non_occ_points(i,2) = non_occ_points(i-1,2);
        x_ch =0; 
    end

    if ((cur_endy ~=y_rob)+(cur_endx ~=x_rob))==0
        break
    end
    
    cur_endy = non_occ_points(i,1);
    cur_endx = non_occ_points(i,2);
    cur_slope = (non_occ_points(i,1)-y_rob)/(non_occ_points(i,2)-x_rob); 
    is_cs_inf = isinf(cur_slope); 
end
non_occ_points