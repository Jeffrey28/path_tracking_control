% Total Center of mass and Moment of inertia calculation (2D)

tire_mass=160;
arm_mass=240;
chassis_mass=4362;
tire_local_J=3.189; % I_xx
link_local_J=15.94; % I_yy
chassis_local_J=7474.2; % I_zz
x_center=0;
y_center=0;
I_z=0;
P=zeros(6,4);
P(1,:)=[2.2017,0.828,tire_mass,tire_local_J];      % FL tire [x,y,kg, local moment of inertia]
P(2,:)=[2.063,0.668,arm_mass,link_local_J];        % FL arm
P(3,:)=[0.6013,0.828,tire_mass,tire_local_J];      % ML tire
P(4,:)=[0.737,0.668,arm_mass,link_local_J];        % ML arm
P(5,:)=[-0.9987,0.828,tire_mass,tire_local_J];     % RL tire
P(6,:)=[-0.863,0.668,arm_mass,link_local_J];       % RL arm
P(7,:)=[P(1,1),-P(1,2),P(1,3),P(1,4)];       % FR tire [x,y,kg]
P(8,:)=[P(2,1),-P(2,2),P(2,3),P(2,4)];       % FR arm
P(9,:)=[P(3,1),-P(3,2),P(3,3),P(3,4)];       % MR tire
P(10,:)=[P(4,1),-P(4,2),P(4,3),P(4,4)];      % MR arm
P(11,:)=[P(5,1),-P(5,2),P(5,3),P(5,4)];      % RR tire
P(12,:)=[P(6,1),-P(6,2),P(6,3),P(6,4)];      % RR arm

for i=1:12
    x_center=x_center+P(i,1)*P(i,3);
    y_center=y_center+P(i,2)*P(i,3);
end

x_center=x_center/(6*(tire_mass+arm_mass)+chassis_mass);
y_center=y_center/(6*(tire_mass+arm_mass)+chassis_mass);
if y_center<0.0001
    y_center=0; % near zero
end

% moment of inertia
for i=1:12
    I_z=I_z+P(i,4)+P(i,3)*((P(i,1)-x_center)^2+(P(i,2)-y_center)^2); % moment of inertia (parallel axis theorem)
end
I_z=I_z+chassis_local_J+chassis_mass*(x_center^2+y_center^2);
L_f=P(1,1)-x_center;
L_m=P(3,1)-x_center;
L_r=P(5,1)-x_center;