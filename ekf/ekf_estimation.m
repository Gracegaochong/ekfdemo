function [xEst, PEst] = ekf_estimation(xEst, PEst, z, u)

global DT;
global Q;
global R;
xPred = motion_model(xEst, u)
yaw = xPred(3,1);
v = u(1,1);
jF=[1.0, 0.0, -DT * v * sin(yaw), DT * cos(yaw);  
    0.0, 1.0, DT * v * cos(yaw), DT * sin(yaw);
    0.0, 0.0, 1.0, 0.0;
    0.0, 0.0, 0.0, 1.0];  %—≈ø…±»æÿ’Û
PPred = jF * PEst * jF' + Q;

jH = [1, 0, 0, 0;
      0, 1, 0, 0];
 H=[1,0,0,0;0,1,0,0];
 zPred=H*xPred;
 y=z- zPred;
 S = jH * PPred * jH' + R;  
 K = PPred * jH' * inv(S);
 
 xEst = xPred + K * y;
 
 PEst = (eye(length(xEst)) - K * jH) * PPred;