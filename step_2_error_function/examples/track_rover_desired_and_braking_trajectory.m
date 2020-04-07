clear;close all

%% description
%
% Create and plot an example desired and braking trajectory for the rover.
%
% Author: Sean Vaskov
% Created: 06 March 2020
%
%% user parameters
t_f = 2 ;
w_0 = 0 ; % rad/s
psi_end = 0;
v_des = 1 ; % m/s
t_plan = 0.5; %s
t_stop = 3; %s


% create roveragent
RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;
l = A.wheelbase;
lr = A.rear_axel_to_center_of_mass;

%initial condition
v0 = 2.0;
w0 = 0;

z0 = [0;0;0;v0;w0];
z0(5)= A.yawrate_to_wheelangle(v0,w0);

T = 1.5;
%% automated from her

[Tref,Uref,Zref] = make_rover_desired_trajectory(t_f,w_0,psi_end,v_des) ;
[Tbrk,Ubrk,Zbrk] = make_rover_braking_trajectory(t_plan,t_f,t_stop,w_0,psi_end,v_des);



%% have agent track trajectory
A.reset(z0)
A.move(T,Tref,Uref,Zref);

Zref = match_trajectories(A.time,Tref,Zref);
Tref = A.time;

subplot(2,2,1)
hold on
plot(A.state(1,:),A.state(2,:),'b','LineWidth',1.0)
subplot(2,2,2)
hold on
plot(A.time,A.state(3,:),'b','LineWidth',1.0)
subplot(2,2,3)
hold on
plot(A.time,A.state(4,:),'b','LineWidth',1.0)
subplot(2,2,4)
hold on
plot(A.time,A.state(5,:),'b','LineWidth',1.0)

A.reset(z0)
A.move(Tbrk(end),Tbrk,Ubrk,Zbrk)
Zbrk = match_trajectories(A.time,Tbrk,Zbrk);
Tbrk = A.time;

subplot(2,2,1)
plot(A.state(1,:),A.state(2,:),'r','LineWidth',1.0)
subplot(2,2,2)
plot(A.time,A.state(3,:),'r','LineWidth',1.0)
subplot(2,2,3)
plot(A.time,A.state(4,:),'r','LineWidth',1.0)
subplot(2,2,4)
plot(A.time,A.state(5,:),'r','LineWidth',1.0)



% plot reference trajectoryies
% get the states of the trajectory
x =   Zref(1,:) ;
y =   Zref(2,:) ;
psi = Zref(3,:) ;
v =   Zref(4,:) ;
delta =  Zref(5,:) ;


% get the states of the braking trajectory
xbrk = Zbrk(1,:) ;
ybrk = Zbrk(2,:) ;
psibrk = Zbrk(3,:) ;
vbrk = Zbrk(4,:) ;
deltabrk =  Zbrk(5,:) ;



set(gca,'FontSize',15)
subplot(2,2,1)
plot(x,y,'b--','LineWidth',1.0)
hold on
plot(xbrk,ybrk,'r--','LineWidth',1.0)
xlabel('x [m]')
ylabel('y [m]')


subplot(2,2,2)
plot(Tref,psi,'b--','LineWidth',1.0)
hold on
plot(Tbrk,psibrk,'r--','LineWidth',1.0)
xlabel('t [s]')
ylabel('\psi [rad]')

subplot(2,2,3)
plot(Tref,v,'b--','LineWidth',1.0)
hold on 
plot(Tbrk,vbrk,'r--','LineWidth',1.0)
xlabel('t [s]')
ylabel('v [m/s]')

subplot(2,2,4)
plot(Tref,delta,'b--','LineWidth',1.0)
hold on
plot(Tbrk,deltabrk,'r--','LineWidth',1.0)
xlabel('t [s]')
ylabel('\delta [rad]')
legend({'desired traj.','braking traj.'})
