function [T,U,Z] = make_segway_braking_trajectory(t_plan,t_stop,w_des,v_des)
% [T,U,Z] = make_segway_braking_trajectory(t_plan,t_stop,w_des,v_des)
%
% Create a Dubins path with braking as a full-state trajectory for the
% TurtleBot.
%
% The inputs are:
%   t_plan   planning timeout
%   t_f      planning time horizon
%   t_stop   duration required for robot to come to a stop
%   w_des    desired yaw rate
%   v_des    desired speed
%
% The outputs are:
%   T        timing for desired trajectory as a 1-by-N array
%   U        desired input (yaw rate and acceleration) as 2-by-N array
%   Z        desired trajectory (x,y,h,w,v) as a 5-by-N array
%
% Note, this is identical to the turtlebot braking trajectory from the RTD
% tutorial.
%
% Author: Shreyas Kousik
% Created: 9 Mar 2020
% Updated: -

    % set up timing
    t_sample = 0.01 ;
    t_total = t_plan + t_stop ;
    T = unique([0:t_sample:t_total,t_total]);

    % create braking traj. for w and v
    t_log = T >= t_plan ;
    braking_scale_power = 4 ;
    scale = ones(size(T)) ;
    scale(t_log) = ((t_stop - T(t_log) + t_plan)./(t_stop)).^braking_scale_power ;

    % get inputs for desired trajectories
    w_traj = w_des.*scale ;
    v_traj = v_des.*scale ;
    U_in = [w_traj ; v_traj] ;

    % compute desired trajectory
    z0 = zeros(3,1) ;
    [~,Z] = ode45(@(t,z) segway_trajectory_producing_model(t,z,T,U_in),T,z0) ;

    % append velocity to (x,y,h) trajectory to make it a full-state
    % trajectory for the segway
    Z = [Z' ; w_traj ; v_traj] ;

    % we put zero feedforward input
    U = zeros(2,length(T)) ;
end