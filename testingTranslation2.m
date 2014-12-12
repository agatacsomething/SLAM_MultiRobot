function [startidx, best_shift] = testingTranslation2(log_prob_4,robot,log_prob2,robot_test_num,start_pt, yshiftnew, spread)
global MAP;
% initvarsometest2;

%[log_prob_4] = buildMapGeneral(500,4,log_prob2,MAP, robot,0);
%[log_prob_2] = buildMapGeneral(300,2,log_prob2,MAP, robot,5);

[bwwalls4] = getWalls3(log_prob_4);
%bwwalls4_noise = imnoise(bwwalls4,'Gaussian');

%spread= -1:0.01:0;

for j=1:length(spread) %length(thetas)
    j
	[log_prob_2] = buildMapGeneral4(start_pt,robot_test_num,log_prob2, robot,yshiftnew, spread(j));
    bwwalls_temp2 = getWalls3(log_prob_2);
	best_shift(j) = corr2(bwwalls4,bwwalls_temp2);

end

[val, idx]=max(best_shift);
startidx=spread(idx); 

%[log_prob_2_new] = buildMapGeneral(300,2,log_prob2,MAP, robot,idx);

% here4= mat2gray(log_prob_4);
% figure, imshow(here4); title('num4')
% 
% here2= mat2gray(log_prob_2_new);
% figure, imshow(here2); title('num2')
% 
% here42= mat2gray(log_prob_4+log_prob_2_new);
% figure, imshow(here42); title('shift num42')

