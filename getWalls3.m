function [bwwalls_new_rev] = getWalls3(log_prob_452)

wallstest= find(log_prob_452>0.1);
bwwallstest= zeros(size(log_prob_452)); 
bwwallstest(wallstest)= 1;
%figure; imshow(bwwallstest);title('walls1')

 
%figure; imshow(bwwallstest_thick);title('walls1')

CC= bwconncomp(bwwallstest);
L= labelmatrix(CC); 
%figure; imagesc(L)
areastemp= regionprops(L,'Area'); 
areas= struct2array(areastemp); 

numareas=0;
newlist= 0; 
for i=1:length(areas)
    if areas(i)>50
        newlist= [newlist; CC.PixelIdxList{i}];
        numareas= numareas+1; 
    end
end

bwwalls_new_rev= zeros(size(log_prob_452)); 
newlist= newlist(2:end); 
k=1; 
newlist= unique(sort(newlist,1,'ascend'));

[I,J] = ind2sub(size(bwwalls_new_rev),newlist);
%bwwalls_new(I,J)=1;

% for j=1:size(bwwalls, 2)
%     for i =1:size(bwwalls,1)
%         if (((i-1)*size(bwwalls, 1) +j) == newlist(k)); 
%             bwwalls_new (j,i)= 1; 
%             k=k+1; 
%         end
%     end
% end


for j=1:size(I, 1) 
	bwwalls_new_rev (I(j),J(j))= 1; 
end

bwwalls_new_rev= imdilate(bwwalls_new_rev, ones(3));



% herewalls_rev= mat2gray(bwwalls_new_rev);
% figure, imshow(herewalls_rev); title('herewalls_rev')

%hold on; plot(-y_robot_new,x_robot_new,'r');

