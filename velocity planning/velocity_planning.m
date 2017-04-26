% f(x) -> v(t) ��ȯ ����

% f(x)=(x-1)^2+1 �̶� ����
% f'(x)=2*x-2
WP=load('Path_rotated.txt');

data_range=30; 
% 
% for i=1:100;
%     data=WP(i:i+data_range-1,:);   % [x,y,v]
%     for j=1:data_range
%         x(j)=sqrt((data(j+1,1)-data(j,1))^2+(data(j+1,2)-data(j,2))^2);
%         v(j)=data(j,3);
%     end
% end
x=0:0.1:4;      % x�� �Ÿ�
f=(x-1).^2+1;   % f�� �Ÿ��� ���� �ӵ� f(x)

% final=length(x);
t_span=[0 2];
Z0=[x(1),f(1)];
% ODE45
[t2,Z_ode]=ode45(@fn_state,t_span,Z0);

figure(1)
subplot(2,1,1)
plot(x,f)
hold on
plot(Z_ode(:,1),Z_ode(:,2),'r')
title('v(x)')
xlabel('distance [m]')
ylabel('velocity [m/s]')
legend('original','after conversion')

subplot(2,1,2)
plot(t2,Z_ode(:,2),'r')
title('v(x) to v(t)')
xlabel('time [s]')
ylabel('velocity [m/s]')