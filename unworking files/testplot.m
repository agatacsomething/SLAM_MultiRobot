   xism=robot{1}.packet{1}.hlidar.xs;
   yism=robot{1}.packet{1}.hlidar.ys;

for i=2:2000
   xis2=robot{1}.packet{i}.hlidar.xs;
   yis2=robot{1}.packet{i}.hlidar.ys;
   
   xism=[xism; xis2];
   yism=[yism; yis2];
   
   
end

figure; scatter(xism, yism)