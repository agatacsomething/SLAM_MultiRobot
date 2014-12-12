chararray = ['1','2','3','4','5'];

for i=1:5
    [pose] = convertStructData(i, robot);
    save(['robot_', chararray(i),'.mat'], 'pose');
end