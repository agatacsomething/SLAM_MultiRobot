% 5:-6, 2:14, 3:7, 1:21
%load('rob4_log_prob2.mat');

tic
% [log_prob_45] = buildMapGeneral3(1900,5,blank_log_prob, robot,-.6);
% 
% [log_prob_2] = buildMapGeneral3(1900,4,log_prob_45, robot,0);

num=1
tic
[log_prob_5v5] = buildMapGeneral4(1900,5,blank_log_prob, robot,-0.6,0,0);
toc
here= mat2gray(log_prob_5v5);
figure, imshow(here); title('log_prob_5v6,3')
load chirp
sound(y,Fs)

num=2
tic
[log_prob_54v5] = buildMapGeneral4(1900,4,log_prob_5v5, robot,0,0,1);
toc 
here= mat2gray(log_prob_54v5);
figure, imshow(here); title('log_prob_54v6-50,7.5,3')
load chirp
sound(y,Fs)

num=3
tic
[log_prob_542v5] = buildMapGeneral4(1600,2,log_prob_54v5, robot,1.4,0,1);
toc
here= mat2gray(log_prob_542v5);
figure, imshow(here); title('log_prob_542v6-30,7.5,3')
load chirp
sound(y,Fs)

num=4
tic
[log_prob_5423v5] = buildMapGeneral4(1900,3,log_prob_542v5, robot,0.7,0,1);
toc
here= mat2gray(log_prob_5423v5);
figure, imshow(here); title('log_prob_5423v6')
load chirp
sound(y,Fs)

num=5
tic
[log_prob_54231v5] = buildMapGeneral4(1900,1,log_prob_5423v5, robot,2.1,0,1);
toc

toc

here= mat2gray(log_prob_54231v5);
figure, imshow(here); title('log_prob_54231v6')

load handel
sound(y,Fs)