posY2 = [xis; yis; zeros(length(yis),1)]';

% x_rob2 = ceil((xpos_g(p) - MAP.xmin) ./ MAP.res);
% y_rob2 = ceil((ypos_g(p) - MAP.ymin) ./ MAP.res);

x_range2 = -100:5:100;
y_range2 = -100:5:100;

c2(:,:,p) = map_correlation(int8(log_prob)...
    ,x_im,y_im,posY,x_range2,y_range2);

figure; surf(c2(:,:,p))

%[x,y,z] = max(max(c(:,:,p)))

[b m2] = max(max(c2(:,:,p)));
[I,J] = find(c2(:,:,p)==b);
x_range2(I)
y_range2(J)


[h1,h2]=min(pdist2([I,J],[21,21]));
cor_pos(n,1) = I(h2)+xpos_g(n+1);
cor_pos(n,2) = J(h2)+xpos_g(n+1);

% %coordinates:
% 
% x_max = x(c(:,:,p)==max_height)
% y_max = y(c(:,:,p)==max_height)


