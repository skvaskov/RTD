%% description
%
% Create and plot an example desired and braking trajectory for the segway
%
% Author: Shreyas Kousik
% Created: 9 Mar 2020
% Updated: 10 Mar 2020
%
%% user parameters
% robot initial condition
w_0 = 0.0 ; % rad/s
v_0 = 0.75 ; % m/s

% trajectory parameters
w_des = 1.0 ; % rad/s
v_des = 1.25 ; % m/s
t_plan = 0.5 ; % s
t_f = 1 ; % for non-braking trajectory

%% automated from here
% make segway for each trajectory
A_go = segway_agent() ;
A_brk = segway_agent() ;

% set the agents to the initial condition
z_0 = [0;0;0;w_0;v_0] ;
A_go.reset(z_0) ;
A_brk.reset(z_0) ;

% get stopping time
t_stop = A_go.max_speed ./ A_go.max_accel ;

% make desired trajectory
[T_go,U_go,Z_go] = make_segway_desired_trajectory(t_f,w_des,v_des) ;

% make braking trajectory
[T_brk,U_brk,Z_brk] = make_segway_braking_trajectory(t_plan,t_stop,w_des,v_des) ;

% track the desired trajectory
A_go.move(t_f,T_go,U_go,Z_go) ;
A_brk.move(t_plan+t_stop,T_brk,U_brk,Z_brk) ;

%% plotting
figure(1) ; clf

%% plot the non-braking trajectory
subplot(3,2,1) ; axis equal ; hold on ; grid on ;
plot(A_go) ;
plot_path(Z_go(1:2,:),'b--','linewidth',1.5)
xlabel('x [m]')
ylabel('y [m]')
title('non-braking traj')
set(gca,'FontSize',15)

subplot(3,2,3) ; hold on ; grid on
plot(T_go,Z_go(4,:)','b--')
plot(A_go.time,A_go.state(4,:)','b')
xlabel('time')
ylabel('yaw rate')
axis([T_go(1) T_go(end) -(A_go.max_yaw_rate+0.2) (A_go.max_yaw_rate+0.2)])
set(gca,'FontSize',15)

subplot(3,2,5) ; hold on ; grid on
plot(T_go,Z_go(5,:)','b--')
plot(A_go.time,A_go.state(5,:)','b')
xlabel('time')
ylabel('speed')
axis([T_go(1) T_go(end) 0 (A_go.max_speed+0.2)])
set(gca,'FontSize',15)

%% plot the braking trajectory
subplot(3,2,2) ; axis equal ; hold on ; grid on ;
plot(A_brk) ;
plot_path(Z_brk(1:2,:),'r--','linewidth',1.5)
xlabel('x [m]')
ylabel('y [m]')
title('braking traj')
set(gca,'FontSize',15)

subplot(3,2,4) ; hold on ; grid on
plot(T_brk,Z_brk(4,:)','r--')
plot(A_brk.time,A_brk.state(4,:)','b')
xlabel('time')
ylabel('yaw rate')
axis([T_brk(1) T_brk(end) -(A_brk.max_yaw_rate+0.2) (A_brk.max_yaw_rate+0.2)])
set(gca,'FontSize',15)

subplot(3,2,6) ; hold on ; grid on
plot(T_brk,Z_brk(5,:)','r--')
plot(A_brk.time,A_brk.state(5,:)','b')
xlabel('time')
ylabel('speed')
axis([T_brk(1) T_brk(end) 0 (A_brk.max_speed+0.2)])
set(gca,'FontSize',15)