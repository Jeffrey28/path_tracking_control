% Path Tracking Control
clc; clear; close all;

global M_c M_c_actual M_d m F_xd F_yd;
% vehicle parameter
m=6762;             % mass(kg)
L=1;             % vehicle length (m)
t_w=1.948;         % distance between right and left wheels (m) = tread
I_z=13201;         % moment of inertia of z-axis (kgm^2)
L_f=1.8788; L_m=0.2784; L_r=1.3216;    % perpendicular length from C.G. to each axle
tire_radius=0.547;            % tire radius (m)

% [X,Y,vx,vy,yaw,yaw_d]'=[Z1,Z2,Z3,Z4,Z5,Z6]'   state
%Z(1)=0; Z(2)=0;         % vehicle position (X, Y) from global coordinate
%Z(3)=0; Z(4)=0;         % vehicle velocity (vx, vy) from vehicle coordinate
%Z(5)=0; Z(6)=0;         % vehicle heading angle (rad, rad/s) from global coordinate
Z=[3.4692383e+01,1.3641483e+02,0,0,0,0];
% Z=[0,150,0,0,0,0];
% control gains
K_vp=10;
K_vi=0;
K_yd=10;
K_yp=20;

% observer gains
P=10;
l=P*I_z;
eta=25*I_z;

% observer initial value
r_hat=0;
M_d_hat=0;
M_d=0;
M_c=0;

% initial conditions
T=0.01;    % step time
e_v=0;      % velocity error
e_v_sum=0;  % velocity error integration
e_y=0;      % yaw error
e_yd=0;     % yaw rate error
L_pv=0;     % preview distance (m)
e_l=0;      % lateral position error (m)

F_y=zeros(1,6);                                             % lateral tire force (front, middle, rear)
C_a=17453;                                                  % cornering stiffness (front, middle, rear)
alpha=zeros(1,6);                                           % side slip angle
W1=1; W2=1; W3=1; W4=1; W5=1; W6=1;                         % weighting coefficient of each wheel
F_z1=m/6*9.81; F_z2=m/6*9.81; F_z3=m/6*9.81;
F_z4=m/6*9.81; F_z5=m/6*9.81; F_z6=m/6*9.81;                % estimated vertical tire force
acc_y=0;                                                    % vehicle total lateral acceleration
F_x_des=zeros(6,1);

% force limit parameter
torque_limit=3416;  % Nm
force_limit=torque_limit/tire_radius;     % gear ratio= 1:1

% waypoints assignment
WP=load('Path_rotated.txt');       % waypoint [x;y;velocity]
i=2;
i2=1;
[WP_size,~]=size(WP);
final_time=500;
Z_save(i2,:)=[0,Z];
error_save(i2,:)=[0,0,0,e_v,0,0,e_y,e_l];
torque_save(i2,:)=[0,0,0,0,0,0,0];
slip_angle_save(i2,:)=[0,0,0,0,0,0,0];
M_d_save(i2,:)=[0,0,0,0,0];
M_c_save(i2,:)=[0,0,0];
F_xd_save(i2,:)=[0,0];
moment(i2,:)=[0,0,0,0,0];
u_wp=0;
yaw_d=0;
% iteration
for t=0:T:final_time
    WP1=WP(i-1,:);              % waypoint 새로 할당
    WP2=WP(i,:);
    dn=sqrt((WP2(1)-WP1(1))^2+(WP2(2)-WP1(2))^2);       % waypoint 거리
    yaw_wp=atan2((WP2(2)-WP1(2)),(WP2(1)-WP1(1)));      % waypoint direction (rad)
    u_wp=((Z(1)-WP1(1))*(WP2(1)-WP1(1))+(Z(2)-WP1(2))*(WP2(2)-WP1(2)))/dn^2;
    while u_wp>1           % u_wp>1 : 차량이 주어진 두 waypoint 범위를 벗어남 -> 새로운 waypoint 할당
        if i==(WP_size)   % 주어진 경로가 더이상 없으면 종료
            break;
        end
        i=i+1;
        processInPercent=i/WP_size*100;
        fprintf('Process in percent: %.2f%% , Velocity : %1.1f , e_l : %1.2f \n',processInPercent, Z(3), e_l);
        WP1=WP(i-1,:);              % waypoint 새로 할당
        WP2=WP(i,:);
        dn=sqrt((WP2(1)-WP1(1))^2+(WP2(2)-WP1(2))^2);
        yaw_wp=atan2((WP2(2)-WP1(2)),(WP2(1)-WP1(1)));      % waypoint direction (rad)
        u_wp=((Z(1)-WP1(1))*(WP2(1)-WP1(1))+(Z(2)-WP1(2))*(WP2(2)-WP1(2)))/dn^2;
    end
    if i==(WP_size)   % 주어진 경로가 더이상 없으면 종료
        break;
    end
    x_d = WP1(1)+u_wp*(WP2(1)-WP1(1));    % desired x position (perpendicular point from the path)
    y_d = WP1(2)+u_wp*(WP2(2)-WP1(2));    % desired y position (perpendicular point from the path)
    if u_wp > 0
        v_d = WP1(3)+u_wp*(WP2(3)-WP1(3));    % desired velocity
    else
        v_d = WP1(3);   % 차량 초기 위치가 waypoint 보다 한참 뒤에 있을 경우 u_wp < 0가 되어 마이너스 속도(v_d < 0) 발생하는 것 방지
    end

    v_d = v_d*exp(-0.5*abs(e_l));   % 횡 방향 위치 오차가 커지면 속도 명령을 줄임 (2016.11.10)
    
    % preview distance
    if abs(Z(3))<L
        L_pv=L;
    else
        L_pv=abs(Z(3))*L;               % preview distance update
    end
    i_pv=i;
    d_rest=sqrt((WP(i_pv,1)-x_d)^2+(WP(i_pv,2)-y_d)^2);
    if L_pv > d_rest
        L_pv_rest=L_pv-d_rest;
        while(L_pv_rest>0)
            if i_pv==(WP_size)
                break;
            end
            i_pv=i_pv+1;
            d_rest=sqrt((WP(i_pv,1)-WP(i_pv-1,1))^2+(WP(i_pv,2)-WP(i_pv-1,2))^2);
            L_pv_rest=L_pv_rest-d_rest;
        end
        L_pv_rest=d_rest+L_pv_rest;
        yaw_wp_pv=atan2((WP(i_pv,2)-WP(i_pv-1,2)),(WP(i_pv,1)-WP(i_pv-1,1)));
        x_pv=WP(i_pv-1,1)+L_pv_rest*cos(yaw_wp_pv);
        y_pv=WP(i_pv-1,2)+L_pv_rest*sin(yaw_wp_pv);
        if i_pv==(WP_size)
            x_pv=WP(i_pv,1);
            y_pv=WP(i_pv,2);
        end
    else
        x_pv=x_d+L_pv*cos(yaw_wp);
        y_pv=y_d+L_pv*sin(yaw_wp);
    end
    
    % lateral position error
    e_l=sqrt((x_d-Z(1))^2+(y_d-Z(2))^2)*sign(y_d-Z(2));
    if abs(e_l)>20
        break;      % 경로에서 너무 많이 벗어날 경우 시뮬레이션 중지
    end
    yaw_d_before = yaw_d;
    yaw_d=atan2((y_pv-Z(2)),(x_pv-Z(1)));    % desired heading angle
    
    % error
    yaw=Z(5);
    if yaw>=0
        if yaw_d<(yaw-pi)
            e_y=yaw_d+2*pi-yaw;
        else
            e_y=yaw_d-yaw;
        end
    else
        if yaw_d>(yaw+pi)
            e_y=(yaw_d-2*pi)-yaw;
        else
            e_y=yaw_d-yaw;
        end
    end
    
    if yaw_d_before>=0
        if yaw_d<(yaw_d_before-pi)
            e_ydes=yaw_d+2*pi-yaw_d_before;
        else
            e_ydes=yaw_d-yaw_d_before;
        end
    else
        if yaw_d>(yaw_d_before+pi)
            e_ydes=(yaw_d-2*pi)-yaw_d_before;
        else
            e_ydes=yaw_d-yaw_d_before;
        end
    end
    
    yaw_d_dot = e_ydes/T;
    e_yd = -Z(6);
    e_v=v_d-Z(3);
    e_v_sum=e_v_sum+e_v*T;
    
    error_save(i2,:)=[t,v_d,Z(3),e_v,yaw_d,Z(5),e_y,e_l];
    
    % tire slip angle (front, middle, rear)
    if Z(4)==0
        alpha=zeros(1,6);
    else
        % sign convention
        alpha(1)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))+Z(6)*L_f*cos(Z(5))-Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))-Z(6)*L_f*sin(Z(5))-Z(6)*t_w/2*cos(Z(5))));
        alpha(3)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))+Z(6)*L_m*cos(Z(5))-Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))-Z(6)*L_m*sin(Z(5))-Z(6)*t_w/2*cos(Z(5))));
        alpha(5)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))-Z(6)*L_r*cos(Z(5))-Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))+Z(6)*L_r*sin(Z(5))-Z(6)*t_w/2*cos(Z(5))));
        
        alpha(2)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))+Z(6)*L_f*cos(Z(5))+Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))-Z(6)*L_f*sin(Z(5))+Z(6)*t_w/2*cos(Z(5))));
        alpha(4)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))+Z(6)*L_m*cos(Z(5))+Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))-Z(6)*L_m*sin(Z(5))+Z(6)*t_w/2*cos(Z(5))));
        alpha(6)=-Z(5)+atan2((Z(3)*sin(Z(5))+Z(4)*cos(Z(5))-Z(6)*L_r*cos(Z(5))+Z(6)*t_w/2*sin(Z(5))),(Z(3)*cos(Z(5))-Z(4)*sin(Z(5))+Z(6)*L_r*sin(Z(5))+Z(6)*t_w/2*cos(Z(5))));
    end
    
    % slip angle sign convention
    for i3=1:6
        if alpha(i3) > pi
            alpha(i3) = alpha(i3) - 2*pi;
        elseif alpha(i3) < -pi
            alpha(i3) = alpha(i3) + 2*pi;
        end
    end    
    
    % lateral tire force with respect to its slip angle
    for i3=1:6
        if abs(alpha(i3)>0.09)  % if slip angle > 0.09 rad, then slip angle = 0.09 fix
            F_y(i3)=-C_a*0.09*sign(alpha(i3));
        else
            F_y(i3)=-C_a*alpha(i3);
        end
    end
    
    % vehicle total lateral force
    F_yd=F_y(1)+F_y(2)+F_y(3)+F_y(4)+F_y(5)+F_y(6);    
    
    % vehicle moment (M_d, M_c)
    M_d=L_f*(F_y(1)+F_y(2))+L_m*(F_y(3)+F_y(4))-L_r*(F_y(5)+F_y(6));
    %M_d=0;
    M_c=I_z*(K_yd*e_yd+K_yp*e_y); 
    
    % vehicle desired longitudinal force
    F_xd=m*(K_vp*(e_v)+K_vi*e_v_sum);       % desired vehicle force
    F_xd_save(i2, :) = [t, F_xd];
    
    % vehicle longitudinal force at each wheel
    A=[2*(W1/(F_z1^2 + 1)+W5/(F_z5^2 + 1)),0,2*W5/(F_z5^2 + 1),0;
        0,2*(W2/(F_z2^2 + 1)+W6/(F_z6^2 + 1)),0,2*W6/(F_z6^2 + 1);
        2*W5/(F_z5^2 + 1),0,2*(W3/(F_z3^2 + 1)+W5/(F_z5^2 + 1)),0;
        0,2*W6/(F_z6^2 + 1),0,2*(W4/(F_z4^2 + 1)+W6/(F_z6^2 + 1))];
    B=[W5/(F_z5^2 + 1)*F_xd-2*W5/(F_z5^2 + 1)*M_c/t_w;
        W6/(F_z6^2 + 1)*F_xd+2*W6/(F_z6^2 + 1)*M_c/t_w;
        W5/(F_z5^2 + 1)*F_xd-2*W5/(F_z5^2 + 1)*M_c/t_w;
        W6/(F_z6^2 + 1)*F_xd+2*W6/(F_z6^2 + 1)*M_c/t_w];
    F_x1234=inv(A)*B;               % inv(A)*B
    F_x_des(1)=F_x1234(1);                % longitudinal tire force at each wheel
    F_x_des(2)=F_x1234(2);
    F_x_des(3)=F_x1234(3);
    F_x_des(4)=F_x1234(4);
    F_x_des(5)=F_xd/2-M_c/t_w-F_x_des(1)-F_x_des(3);
    F_x_des(6)=F_xd/2+M_c/t_w-F_x_des(2)-F_x_des(4);
    
    % longitudinal tire force limit
    if abs(F_x_des(1))>force_limit || abs(F_x_des(2))>force_limit || abs(F_x_des(3))>force_limit || abs(F_x_des(4))>force_limit || abs(F_x_des(5))>force_limit || abs(F_x_des(6))>force_limit
        limit_ratio=force_limit/max(abs(F_x_des));
        for j=1:6
            F_x_des(j)=F_x_des(j)*limit_ratio;
        end
    end
    
    M_c_actual=t_w/2*(F_x_des(2)+F_x_des(4)+F_x_des(6)-F_x_des(1)-F_x_des(3)-F_x_des(5));  % actual M_c
    % disturbance moment observer
    r_hat=r_hat+T/I_z*M_d_hat+T/I_z*M_c+T*P*(Z(6)-r_hat);   % yaw rate estimate
    M_d_hat=M_d_hat+T*eta*(Z(6)-r_hat);                     % disturbance moment estimate    
    
    % data save for plot
    M_d_save(i2,:)=[t,M_d,M_d_hat,Z(6),r_hat];     % [time, M_d_true, M_d_estimate, yaw rate, yaw rate estimate]
    M_c_save(i2,:)=[t,M_c,M_c_actual];
    %YSM=abs(yaw_d_dot)-abs(Z(6));
    %moment(i2,:)=[t,M_c,M_d_hat, sign(M_c*M_d_hat), YSM];
    torque_save(i2,:)=[t,F_x_des(1)*tire_radius,F_x_des(2)*tire_radius,F_x_des(3)*tire_radius,F_x_des(4)*tire_radius,F_x_des(5)*tire_radius,F_x_des(6)*tire_radius];
    slip_angle_save(i2,:)=[t,alpha.*180/pi];
    % ODE45
    [t2,Z_ode]=ode45(@fn_state,[0 T],Z);
    [N,M]=size(t2);
    Z=Z_ode(N,:);
    i2=i2+1;
    Z_save(i2,:)=[t,Z];
end

% plot
figure(1)   % path plot
plot(WP(:,1),WP(:,2),'--b','LineWidth',2)
hold on
plot(Z_save(:,2),Z_save(:,3),'r','LineWidth',1)
title('Vehicle Path Tracking')
xlabel('X (m)')
ylabel('Y (m)')
legend('Desired','Simulated','Location','NorthEast')
%axis([-700 0 0 700])
grid on

figure(2)   % error plot
subplot(2,1,1)
plot(error_save(:,1),error_save(:,2),'--b',error_save(:,1),error_save(:,3),'r',error_save(:,1),error_save(:,4),'k')
title('Longitudinal Velocity')
xlabel('time(sec)')
ylabel('Velocity')
legend('Desired','Simulated','Error')
grid on
subplot(2,1,2)
plot(error_save(:,1),error_save(:,5).*180/pi,'--b',error_save(:,1),error_save(:,6).*180/pi,'r',error_save(:,1),error_save(:,7).*180/pi,'k')
title('Heading Angle')
xlabel('time(sec)')
ylabel('Angle')
legend('Desired','Simulated','Error')
grid on

figure(3)   % torque input
plot(torque_save(:,1),torque_save(:,2),':k')
hold on
plot(torque_save(:,1),torque_save(:,3))
title('Torque Input (Front tire)')
xlabel('time(sec)')
ylabel('Torque (Nm)')
legend('FL','FR')
grid on

figure(4)   % side slip angle
subplot(2,1,1)
plot(slip_angle_save(:,1),slip_angle_save(:,2),':k')
hold on
plot(slip_angle_save(:,1),slip_angle_save(:,4),'--r')
hold on
plot(slip_angle_save(:,1),slip_angle_save(:,6))
title('Side Slip Angle')
xlabel('time(sec)')
ylabel('Angle (degree)')
legend('FL','ML','RL')
grid on
subplot(2,1,2)
plot(Z_save(:,1),Z_save(:,7),'k')
title('Yaw Rate')
xlabel('time(sec)')
ylabel('Yaw rate')
legend('yaw rate')
grid on

figure(5)   % disturbance moment
plot(M_d_save(:,1),M_d_save(:,2),'--k')
hold on
plot(M_d_save(:,1),M_d_save(:,3),'k')
title('Disturbance Moment')
xlabel('time(sec)')
ylabel('Disturbance moment (Nm)')
legend('Simulated','Estimated')
grid on

figure(5)   % disturbance moment
plot(M_d_save(:,1),M_d_save(:,2),'--k')
hold on
plot(M_d_save(:,1),M_d_save(:,3),'b')
title('Disturbance Moment Estimation')
xlabel('time(sec)')
ylabel('Disturbance moment (Nm)')
legend('M_d true','M_d estimate')
grid on

figure(6)   % lateral position error
plot(error_save(:,1),error_save(:,8))
title('Lateral Position Error')
xlabel('time(sec)')
ylabel('Lateral position error (m)')
grid on

figure(7)   % yaw rate estimation
plot(M_d_save(:,1),M_d_save(:,4),':b')
hold on
plot(M_d_save(:,1),M_d_save(:,5),'r')
title('Yaw Rate')
xlabel('time(sec)')
ylabel('Yaw rate')
legend('r','r hat')
grid on

figure(8)
plot(M_c_save(:,1),M_c_save(:,2))
hold on
plot(M_d_save(:,1),M_d_save(:,2))
title('M_c & M_d')
xlabel('time(sec)')
ylabel('Moment')
%axis([0 600 -6000 6000])
legend('M_c','M_d')
grid on

figure(9)
plot(error_save(:,1),abs(error_save(:,8)))
hold on
plot(error_save(:,1),abs(error_save(:,7)),'k')
title('Lateral position and Heading angle error')
xlabel('time[sec]')
ylabel('[m] or [rad]')
legend('Lateral position','Heading angle error')
grid on

figure(10)
plot(F_xd_save(:,1), F_xd_save(:,2))
title('Desired Longitudinal Force')
xlabel('Time [s]')
ylabel('Force [N]')
grid on