      


m=1:25;
rep=0;
stdv = stdv_real*0.15;
stdven = stdv_real*0.15;
thetag(1)=2.34312280898684;
xposg(1)=171.0;
yposg(1)=-23.7;
            
for n = m
	[xposg(n+1), yposg(n+1), thetag(n+1)] = ...
    getPossiblePose(xposg(1), yposg(1), thetag(1), stdv, stdven);
end

compos = [xposg;yposg;thetag]'