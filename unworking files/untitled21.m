diff_in_turn(1)=-1;
for i=2:length(big_turns)-1
    diff_in_turn(i) = (big_turns(i) - big_turns(i-1))*180/pi; 
end