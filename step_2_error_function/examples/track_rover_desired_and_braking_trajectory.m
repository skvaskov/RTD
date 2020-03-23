clear

%% description
%
% Create and plot an example desired and braking trajectory for the rover.
%
% Author: Sean Vaskov
% Created: 06 March 2020
%
%% user parameters
t_f = 2 ;
w_0 = 0.7613 ; % rad/s
psi_end = 0.5;
v_des = 1.1821 ; % m/s
t_plan = 0.5; %s
t_stop = 2; %s


% create roveragent
RoverLLC = rover_PD_LLC('yaw_gain',4,'yaw_rate_gain',1);
A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;
l = A.wheelbase;
lr = A.rear_axel_to_center_of_mass;

%initial condition
v0 = 1.7875;
w0 = 0.0086;

z0 = [0;0;0;v0;w0];
z0(5)= A.yawrate_to_wheelangle(v0,w0);


%% automated from her

[T,U,Z] = make_rover_desired_trajectory(t_f,w_0,psi_end,v_des) ;
[Tbrk,Ubrk,Zbrk] = make_rover_braking_trajectory(t_plan,t_f,t_stop,w_0,psi_end,v_des);

% get the states of the trajectory
x = Z(1,:) ;
y = Z(2,:) ;
psi = Z(3,:) ;
v = Z(4,:) ;
delta =  Z(5,:) ;

% get the states of the braking trajectory
xbrk = Zbrk(1,:) ;
ybrk = Zbrk(2,:) ;
psibrk = Zbrk(3,:) ;
vbrk = Zbrk(4,:) ;
deltabrk =  Zbrk(5,:) ;

% plot
figure;
set(gca,'FontSize',15)
subplot(2,2,1)
plot(x,y,'b--','LineWidth',1.0)
hold on
plot(xbrk,ybrk,'r--','LineWidth',1.0)
xlabel('x [m]')
ylabel('y [m]')


subplot(2,2,2)
plot(T,psi,'b--','LineWidth',1.0)
hold on
plot(T,psibrk,'r--','LineWidth',1.0)
xlabel('t [s]')
ylabel('\psi [rad]')

subplot(2,2,3)
plot(T,v,'b--','LineWidth',1.0)
hold on 
plot(T,vbrk,'r--','LineWidth',1.0)
xlabel('t [s]')
ylabel('v [m/s]')

subplot(2,2,4)
plot(T,delta,'b--','LineWidth',1.0)
hold on
plot(T,deltabrk,'r--','LineWidth',1.0)
xlabel('t [s]')
ylabel('\delta [rad]')
legend({'desired traj.','braking traj.'})

%% have agent track trajectory
A.reset(z0)
A.move(t_f,T,U,Z);
subplot(2,2,1)
plot(A.state(1,:),A.state(2,:),'b','LineWidth',1.0)
subplot(2,2,2)
plot(A.time,A.state(3,:),'b','LineWidth',1.0)
subplot(2,2,3)
plot(A.time,A.state(4,:),'b','LineWidth',1.0)
subplot(2,2,4)
plot(A.time,A.state(5,:),'b','LineWidth',1.0)

A.reset(z0)
A.move(t_f,Tbrk,Ubrk,Zbrk)
subplot(2,2,1)
plot(A.state(1,:),A.state(2,:),'r','LineWidth',1.0)
subplot(2,2,2)
plot(A.time,A.state(3,:),'r','LineWidth',1.0)
subplot(2,2,3)
plot(A.time,A.state(4,:),'r','LineWidth',1.0)
subplot(2,2,4)
plot(A.time,A.state(5,:),'r','LineWidth',1.0)




