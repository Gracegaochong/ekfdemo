clc;
clear;

global DT;
global Qsim;
global Rsim;
global Q;
global R;

Q=[0.1 0 0 0;
    0 0.1 0 0;
    0 0 1*3.1415926/180 0;
    0 0 0 0.1]^2;
R=[1 0;
    0 40*3.1415926/180]^2;
    
Qsim=(0.5*eye(2))^2;
Rsim = [1 0;
        0 40*3.141592654/180]^2;
SIM_TIME=50;
DT=0.1;
time=0;

xEst=zeros(4,1);%估计路线
xTrue=zeros(4,1);%真实路线
PEst=eye(4);  %P矩阵
xDIR=zeros(4,1);

hxEkf=[];
hxTrue=[];%真值
hxDIR=[];%drz值
hz=[];
while SIM_TIME >=time
    time=time+DT;
    v=1;
    yawrate=0.1;
    u=[v;yawrate];
    %xTrue真值   %z观测值 添加niose 
    %xDR         %ud 添加noise
    [xTrue,z,xDIR,ud]=observation(xTrue,xDIR,u);
    [xEst, PEst] = ekf_estimation(xEst, PEst, z, ud);
    hxTrue=[hxTrue,xTrue];
    hz=[hz,z];
    hxDIR=[hxDIR xDIR];
    hxEkf=[hxEkf xEst];
    plot(hxTrue(1,:),hxTrue(2,:),'r.') 
    hold on 
    plot(hz(1,:),hz(2,:),'g-') 
    hold on
    plot(hxDIR(1,:),hxDIR(2,:),'b.') 
    hold on 
    plot(hxEkf(1,:),hxEkf(2,:),'k.') 
    plotcov(xEst,PEst);
    pause(0.1)
end

err = hxTrue-hxEkf;
figure('name','error')
hold on
plot(err(1,:),'r.');
plot(err(2,:),'b.');
legend('error-x', 'error-y'); 
grid on
hold off


figure('name','error1')
histogram(err(1,:));
figure('name','error2')
histogram(err(2,:));
m1=mean(err(1,:))
s1=std(err(1,:))

m2=mean(err(2,:)) 
s2=std(err(2,:)) 



