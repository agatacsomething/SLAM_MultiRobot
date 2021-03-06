function [bwwalls_new] = getWalls(log_prob2)

walls= find(log_prob2>0);
bwwalls= zeros(size(log_prob2)); 
bwwalls(walls)= 1; 
CC= bwconncomp(bwwalls);
L= labelmatrix(CC); 
%figure; imagesc(L)
areastemp= regionprops(L,'Area'); 
areas= struct2array(areastemp); 

numareas=0;
newlist= 0; 
for i=1:length(areas)
    if areas(i)>15
        newlist= [newlist; CC.PixelIdxList{i}];
        numareas= numareas+1; 
    end
end

bwwalls_new= zeros(size(log_prob2)); 
newlist= newlist(2:end); 
k=1; 
newlist= unique(sort(newlist,1,'ascend'));

[I,J] = ind2sub(size(bwwalls_new),newlist);
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
	bwwalls_new (I(j),J(j))= 1; 
end

% herewalls= mat2gray(bwwalls_new);
% figure, imshow(herewalls); %hold on; plot(-y_robot_new,x_robot_new,'r');

