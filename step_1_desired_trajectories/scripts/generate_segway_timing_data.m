%% description
% This script generates the timing information for the segway. It's
% basically identical to the turtlebot's case.
%
% Author: Shreyas Kousik
% Created: 10 Mar 2020
% Updated: 11 Mar 2020
%
%% user parameters
% amount of time to spend per planning iteration
t_plan = 0.5 ; % s

% maximum allowed change in the segway's speed
delta_v = 0.5 ; % m/s

% whether or not to save data
save_data_flag = false ;

%% automated from here
% make segway agent
A = segway_agent() ;

% create vector of initial speeds
v_0_vec = 0.1:0.1:A.max_speed ;

% get vector to fill in with distances traveled
d_brk = nan(size(v_0_vec)) ;

% for each initial speed, generate a braking trajectory and then execute it
t_extra = 0.05 ; % a little bit of extra time to make sure we fully stop
idx = 1 ;
for v_0 = v_0_vec
    % get stopping time for current t_stop
    t_stop = v_0/A.max_accel ;
    
    % set up braking trajectory
    w_0 = 0 ;
    w_des = 0 ;
    v_des = v_0 ;
    [T_brk,U_brk,Z_brk] = make_segway_braking_trajectory(t_plan,t_stop,w_des,v_des) ;
    
    % track braking trajectory
    z0 = [0;0;0;w_0;v_0] ;
    A.reset(z0)
    A.move(T_brk(end)+t_extra,T_brk,U_brk,Z_brk) ;
    
    % find the total distance traveled during braking
    T = A.time ;
    d_at_t_plan = match_trajectories(t_plan,A.time,A.state(1,:)) ;
    d_brk(idx) = A.state(1,end) - d_at_t_plan ;
    idx = idx + 1 ;
end

%% compute t_f for each of the max initial speed ranges
% get t_stop for all the braking distances
t_stop_vec = d_brk ./ v_0 ;

% round up to the nearest 0.1 s to preempt numerical errors
t_stop_vec = ceil(10*t_stop_vec)./10 ;

% get t_f for the FRSes we plan to compute later
v_0_all = [0.5 1.0 1.5] ;
v_begin_braking = v_0_all + delta_v.*[1 1 0] ;
t_f_all = match_trajectories(v_begin_braking,v_0_vec,t_stop_vec) + t_plan ;

%% save timing
if save_data_flag
    save('segway_timing.mat','t_plan','t_f_all',...
        'v_0_all','v_begin_braking','delta_v') ;
end

%% plot v_0 vs d
figure(1) ; clf ; hold on ; grid on ;
plot(v_0_vec,d_brk,'LineWidth',1.5)
xlabel('v_0 [m/s]')
ylabel('braking distance [m]')
set(gca,'FontSize',15)