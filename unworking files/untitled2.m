for j=1:length(thetas)
    [log_prob_temp]= getNewTempMap2(ptd3,robot,log_prob2,thetas(j), ...
        robot_num, rob, rob_newyshiftnew,shifts(k),0);
    
    bwwalls_temp = getWalls2(log_prob_temp);
    
    corr_angles(j) = corr2(bwwalls,bwwalls_temp);

end

[val idx]=max(corr_angles);
angleshift= thetas(idx);

for k=1:length(shifts)
        
        [log_prob_temp_y]= getNewTempMap2(ptd3,robot,log_prob2,angleshift, ...
            robot_num, rob, rob_new,yshiftnew,shifts(k),0);
        
        bwwalls_temp_y = getWalls2(log_prob_temp_y);
        corr_angles_yshift(k) = corr2(bwwalls,bwwalls_temp_y);
        
end

[shiftyval, shiftyidx]=max(corr_angles_yshift);
yshift= shifts(shiftyidx);

for p=1:length(shifts)
    
    [log_prob_temp_x]= getNewTempMap2(ptd3,robot,log_prob2,thetas(j), ...
        robot_num, rob, rob_new,yshiftnew,yshift,shifts(p));
    
    bwwalls_temp_x = getWalls2(log_prob_temp_x);

    corr_angles_xshift(k) = corr2(bwwalls,bwwalls_temp_x);

end

[shiftxval, shiftxidx]=max(corr_angles_xshift);
