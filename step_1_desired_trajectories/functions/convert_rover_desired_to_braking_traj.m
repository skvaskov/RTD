function [T_brk,U_brk,Z_brk] = convert_rover_desired_to_braking_traj(t_plan,t_stop,T,U,Z)
% [T_brk,U_brk,Z_brk] = convert_rover_desired_to_braking_traj(t_plan,t_stop,T,U,Z)
%
% Given a desired trajectory without braking, transform it into one with
% braking by commanding a hard stop and linearly decreasing the yaw rate.
% This causes the rover to brake "along" the original desired
% trajectory.
%
% Author: Sean Vaskov
% Created: 6 March 2020

    % initialize the output as a copy of the initial trajectory
    T_brk = T ;
    Z_brk = Z ;
    
    %get the distance of the trajectory
    p = [0,cumsum(sqrt(diff(Z_brk(1,:)).^2+diff(Z_brk(2,:)).^2))];

    % get the part of the trajectory after t_plan as the braking part of
    % the trajectory
    t_log = T >= t_plan ;
    brk_start_idx = find(t_log,1);
    
    decel_rate = 2;
    t_decel = T <= t_plan+(Z_brk(4, brk_start_idx)/decel_rate) & t_log;

    % set the desired velocity to decelerate at the desired decel rate to zero
    v_decel = Z_brk(4,brk_start_idx)-decel_rate*(T(t_decel)-t_plan);
 
    Z_brk(4,t_decel) =  v_decel;
    Z_brk(4,t_log & ~t_decel) = 0;
    v_brk = Z_brk(4,:);
    
    % reinterpolate the timing to be long enough for stopping
    N_t = sum(t_log) ;
    t_new = linspace(t_plan,t_plan+t_stop,N_t) ;
    T_brk(t_log) = t_new ;
    
    
    % back out the new distance from the velocity profile
    pbrk  = [0,cumsum(v_brk(1:end-1).*diff(T_brk))];
    
    Zint = interp1(p,Z_brk',pbrk)';
    Z_brk([1:3,5],:) = Zint([1:3,5],:);
    
    
    % create braking nominal input
    U_brk = Z_brk([4 5],:) ;
    U_brk(1,t_log) = 0;
end