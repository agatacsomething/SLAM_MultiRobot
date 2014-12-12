%idx=14; 
global MAP;
c2=clock;
fix(c2)
tic


%[log_prob_4] = buildMapGeneral(500,4,blank_log_prob,MAP, robot,0);

% [startidx] = testingTranslation(log_prob_4,MAP,robot,blank_log_prob,...
%      5,800); -6
% 5:-6, 2:14, 3:7, 1:21
% load('rob4_log_prob2.mat');
% 
% [log_prob_42] = buildMapGeneral(2000,4,log_prob2,MAP, robot,idx);

% 
% [log_prob_5_new] =  buildMapGeneral(800,5,blank_log_prob,MAP, robot,startidx);
% toc

% [startidx3, best_shift] = testingTranslation(log_prob_4,MAP,robot,blank_log_prob,...
%      3,500); %-6

% [log_prob_3_new] =  buildMapGeneral(500,3,blank_log_prob,MAP, robot,startidx3);
% toc
%[log_prob_4] =  buildMapGeneral4(800,4,blank_log_prob, robot,0, 0);

% spread5= -.8:0.01:-.6;
% [startidx5, best_shift5] = testingTranslation2(log_prob2,MAP,robot,blank_log_prob,...
%      5,800,spread5); 
%  
%  spread2= 1:0.01:1.8;
% [startidx2, best_shift2] = testingTranslation2(log_prob2,MAP,robot,blank_log_prob,...
%      2,500,spread2); 
 
%  spread5= -2:0.1:2;
% [startidx5, best_shift3] = testingTranslation2(log_prob_4,robot,blank_log_prob,...
%      5,800,-.6, spread5); 
% %  
%  spread2= 1.9:0.01:2.3;
% [startidx1, best_shift2] = testingTranslation2(log_prob_4,robot,blank_log_prob,...
%      2,400,1.4, spread2); 
 
  spread1= -1.9:0.1:2;
%[startidx1, best_shift1] = testingTranslation2(log_prob_4,robot,blank_log_prob,...
 %    2,400,1.4, spread1);
 

 %[log_prob_5] =  buildMapGeneral4(800,5,blank_log_prob, robot,-.6,0.1 );
 %[log_prob_2] =  buildMapGeneral4(650,2,blank_log_prob, robot,1.4,-.2);
 [log_prob_1] =  buildMapGeneral4(800,1,log_prob2, robot,2.1,0);
 % toc 

%buildMapGeneral(900,5,robot,MAP, robot,0);

% here4= mat2gray(log_prob_4);
% figure, imshow(here4); title('num4')
% 
% here3= mat2gray(log_prob_5);
% figure, imshow(here3); title('num5')
% 
here43= mat2gray(log_prob2+log_prob_1);
figure, imshow(here43); title('shift num452')


% [log_prob_2] = buildMapGeneral2(1900,4,blank_log_prob,MAP, robot,0);
% 
% [log_prob_45] = buildMapGeneral2(1900,5,log_prob_2,MAP, robot,startidx5);
% 
% [log_prob_452] = buildMapGeneral2(1600,2,log_prob_45,MAP, robot,startidx2);
% 
% [log_prob_4523] = buildMapGeneral2(1900,3,log_prob_452,MAP, robot,startidx3);
% 
% [log_prob_45231] = buildMapGeneral2(1900,1,log_prob_4523,MAP, robot,startidx1);
% 
% here= mat2gray(log_prob_45231);
% figure, imshow(here); title('log_prob_45231')

