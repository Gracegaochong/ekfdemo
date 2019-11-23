clc  
clear all  
 
% 初始化参数  
delta_t = 0.1;    % 采样时间  
t  = 0:delta_t:5; % 时间范围 
N  = length(t);    
sz = [2,N];       %位移 速度
g  = 10;          % 加速度值   
x  = 1/2*g*t.^2;  % 实际真实位置序列 0×t +  1/2×g×t^2
%z  = x + sqrt(10).*randn(1,N); % 仿真的实际测量值 
% 测量时加入测量白噪声 测量噪声v 
% 方差R 为 10  均值为0 
load('z.mat');

Q = [0 0;0 9e-1]; % 系统噪声协方差矩阵 
R = 10;           % 测量协方差估计
%Q = [9e-1 0;0 9e-1];

% n*n 状态 转移 矩阵  
%x = x * 1 + v * delta_t  + 1/2*delta_t^2 *g ;  
%v = x * 0  + v * 1 + delta_t*g
A = [1 delta_t;0 1];         % 系统状态转移矩阵
B = [1/2*delta_t^2;delta_t]; % 系统输入转移矩阵
H = [1,0];                   % m*n  系统测量转移矩阵   z = 1 * x + 0 * v  + 噪声

n = size(Q);  % 系统状态维度数量 n为一个1*2的向量  Q为方阵   
m = size(R);  % 测量变量维度数量 

% 分配空间  
xhat      = zeros(sz);       % x的后验估计
P         = zeros(n);        % 后验方差估计
xhatminus = zeros(sz);       % x的先验估计
Pminus    = zeros(n);        
K         = zeros(n(1),m(1));% 卡尔曼增益   
I         = eye(n);          

% 估计的初始值都为默认的0，即P=[0 0;0 0],xhat=0  
P=[2 0;0 2] %系统初始方差较大 算出来的增益 K 就大，增益K 是测量真值 和 测量预测值 误差的系数 ，所以更相信测量值
psave=P;
for k = 9:N    % 假设车子已经运动9个delta_T了，我们才开始估计   
 % 时间更新过程  
 xhatminus(:,k) = A * xhat(:,k-1) + B*g;                                                 
 Pminus         = A * P * A' + Q;                                                         
 % 测量更新过程  
 K         = Pminus * H' * inv( H * Pminus*H'+ R);                                                                               
 xhat(:,k) = xhatminus(:,k) + K*( z(k) - H * xhatminus(:,k));                                                                                                                                      
 P         = ( I - K * H ) * Pminus;
 psave=[psave,P];
end  

figure('name','kf');
hold on
plot(t(:,9:end),z(:,9:end),'g-');  
plot(t,xhat(1,:),'b-')  
plot(t,x(1,:),'r-');  
plot(t,z,'k.');  
plot(t,xhat(1,:),'k.')  
plot(t,x(1,:),'k.'); 
legend('含有噪声的测量', '后验估计', '真值');  
xlabel('Iteration');  

err = x(1,:)-xhat(1,:);
figure('name','error')
plot(t,err);
m1=mean(err)
s1=std(err)

