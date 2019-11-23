function [xTrue,z,xDR,ud] =observation(xTrue,xDR,u)
%xTrue真值 
%z观测值 添加niose 
%xDR
%ud 添加noise
global Qsim;
global Rsim;

xTrue = motion_model(xTrue,u);

%add noise to gps-xy
zx = xTrue(1,1) + randn()*Qsim(1,1);
zy = xTrue(2,1) + randn()*Qsim(2,2);
z=[zx;zy];

%add noise to input
ud1 = u(1,1) + randn()*Rsim(1,1);
ud2 = u(2,1) + randn()*Rsim(2,2);
ud = [ud1;ud2];

xDR = motion_model(xDR,ud);

