function [log_prob] = updateMap2(log_prob, ocup_points,x_rob,y_rob,xis,yis)
%k=0;
global MAP; 

% xis = ceil((ocup_points(:,1) - MAP.xmin) ./ MAP.res);
% yis = ceil((ocup_points(:,2) - MAP.ymin) ./ MAP.res);

for k = 1:size(ocup_points,1)
%k=k+1;
        

                
        indGood = (xis(k,1) > 1) & (yis(k,1) > 1) & (xis(k,1) < MAP.sizex)...
                & (yis(k,1) < MAP.sizey);
    
            
        if indGood ==1
            %inds = sub2ind(size(MAP.map),xis(k,1),yis(k,1));
%             if log_prob(inds)<=3.01
%                 log_prob(inds) = log_prob(inds)+0.02; 
%                 %log_prob_walls(inds)= 1;
%             end
        if log_prob(xis(k,1),yis(k,1))<=2.01
            log_prob(xis(k,1),yis(k,1)) = log_prob(xis(k,1),yis(k,1))+0.02; 
                %log_prob_walls(inds)= 1;
        end
        
        %for
        
         %   MAP.map(xis(k,1),yis(k,1)) = MAP.map(xis(k,1),yis(k,1))+1;
        
        %[bwwalls] = getWalls3(log_prob);
        
        [ax, by] = getMapCellsFromRay(x_rob, y_rob, double(xis(k,1)), double(yis(k,1)));
                
        %inds2 = sub2ind(size(MAP.map),ax,by);
        
        for ii = 1: length(ax)
            %inds2 = sub2ind(size(MAP.map),ax(ii),by(ii));
%             if log_prob(inds2(ii))>=-3.02 %&& ax(ii)>0 && bx(ii)>0%&& log_prob(ax(ii),by(ii))<=3.03
% 
%             	log_prob(inds2(ii)) = log_prob(inds2(ii))- 0.01;
%             end
            if log_prob(ax(ii),by(ii))>=-2.02

            	log_prob(ax(ii),by(ii)) = log_prob(ax(ii),by(ii))- 0.01;
            end

        end
        end       
        clear ax;
        clear by; 
end