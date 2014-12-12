function [xpos, ypos, theta] = getPossiblePose (xold, yold, thetaold, stdv,stdven)

% My noise model for encoder values i.e. for x/y locations 
% of the robot pose is given by :
enc_constt = 0.005;
enc_prop = stdven;
encoder_noise = (enc_prop*thetaold + enc_constt)*randn(1,1);

xpos = xold + encoder_noise;
ypos = yold + encoder_noise;

% where 
% enc_prop = standard deviation of all the delta theta
% values i.e. change in orientation values for all states
% enc_constt = a constant value that is added to the noise
% (set to 0.005)

% My noise model for orientation of the robot is given by : 

gyro_prop= stdv;
%gyro_constt = pi/150;
gyro_constt = pi/15;
gyro_noise = (gyro_prop*thetaold + gyro_constt)*randn(1,1);

theta = thetaold+gyro_noise; 

%where gyro_prop = standard deviation of all the delta theta 
% values i.e. change in orientation values for all states
% gyro_constt = a constant value that is added to the noise 
%(set to pi/150)


