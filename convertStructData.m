function [pose] = convertStructData(num, robot)

length_route= numel(robot{num}.packet);

for i=1:length_route
    pose(i,1)=robot{num}.packet{i}.pose.x;
    pose(i,2)=robot{num}.packet{i}.pose.y;
    pose(i,3)=robot{num}.packet{i}.pose.z;
    pose(i,4)=robot{num}.packet{i}.pose.yaw;
end