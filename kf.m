clc  
clear all  
 
% ��ʼ������  
delta_t = 0.1;    % ����ʱ��  
t  = 0:delta_t:5; % ʱ�䷶Χ 
N  = length(t);    
sz = [2,N];       %λ�� �ٶ�
g  = 10;          % ���ٶ�ֵ   
x  = 1/2*g*t.^2;  % ʵ����ʵλ������ 0��t +  1/2��g��t^2
%z  = x + sqrt(10).*randn(1,N); % �����ʵ�ʲ���ֵ 
% ����ʱ������������� ��������v 
% ����R Ϊ 10  ��ֵΪ0 
load('z.mat');

Q = [0 0;0 9e-1]; % ϵͳ����Э������� 
R = 10;           % ����Э�������
%Q = [9e-1 0;0 9e-1];

% n*n ״̬ ת�� ����  
%x = x * 1 + v * delta_t  + 1/2*delta_t^2 *g ;  
%v = x * 0  + v * 1 + delta_t*g
A = [1 delta_t;0 1];         % ϵͳ״̬ת�ƾ���
B = [1/2*delta_t^2;delta_t]; % ϵͳ����ת�ƾ���
H = [1,0];                   % m*n  ϵͳ����ת�ƾ���   z = 1 * x + 0 * v  + ����

n = size(Q);  % ϵͳ״̬ά������ nΪһ��1*2������  QΪ����   
m = size(R);  % ��������ά������ 

% ����ռ�  
xhat      = zeros(sz);       % x�ĺ������
P         = zeros(n);        % ���鷽�����
xhatminus = zeros(sz);       % x���������
Pminus    = zeros(n);        
K         = zeros(n(1),m(1));% ����������   
I         = eye(n);          

% ���Ƶĳ�ʼֵ��ΪĬ�ϵ�0����P=[0 0;0 0],xhat=0  
P=[2 0;0 2] %ϵͳ��ʼ����ϴ� ����������� K �ʹ�����K �ǲ�����ֵ �� ����Ԥ��ֵ ����ϵ�� �����Ը����Ų���ֵ
psave=P;
for k = 9:N    % ���賵���Ѿ��˶�9��delta_T�ˣ����ǲſ�ʼ����   
 % ʱ����¹���  
 xhatminus(:,k) = A * xhat(:,k-1) + B*g;                                                 
 Pminus         = A * P * A' + Q;                                                         
 % �������¹���  
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
legend('���������Ĳ���', '�������', '��ֵ');  
xlabel('Iteration');  

err = x(1,:)-xhat(1,:);
figure('name','error')
plot(t,err);
m1=mean(err)
s1=std(err)

