function [log_prob, MAP] = updateMap(log_prob, ocup_points)
%k=0;
global MAP; 

for k = 1:length(ocup_points)
%k=k+1;
        
        xis(k,1) = ceil((ocup_points(k,1) - MAP.xmin) ./ MAP.res);
        yis(k,1) = ceil((ocup_points(k,2) - MAP.ymin) ./ MAP.res);
                
        indGood = (xis(k,1) > 1) & (yis(k,1) > 1) & (xis(k,1) < MAP.sizex)...
                & (yis(k,1) < MAP.sizey);
    
        if indGood ==1
            inds = sub2ind(size(MAP.map),xis(k,1),yis(k,1));
            log_prob(inds) = log_prob(inds)+0.02; 
            MAP.map(inds) = MAP.map(inds)+1;
        end 

               
          
end