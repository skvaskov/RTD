function [T,U,Z] = make_rover_desired_trajectory(t_f,w_0,psi_end,v_des)
% [T,U,Z] = make_rover_desired_trajectory(t_f,w_0,psi_end,v_des)
%
% Create a Lange Change as a full-state trajectory for the TurtleBot.
%
% The inputs are:
%   t_f      planning time horizon
%   w_0      desired_initial_yaw rate
%   psi_end  desired heading at t_f
%   v_des    desired_speed
%
% The outputs are:
%   T        timing for desired trajectory as a 1-by-N array
%   U        desired input (velocity command and wheel angle) as 2-by-N array
%   Z        desired trajectory (x,y,h,v,delta) as a 5-by-N array
%
% Author: Sean Vaskov
% Created: 06 March 2020
    % distance from the front and rear wheels to center of mass
     lf = 0.25;
     lr = 0.0765;
     l = lf + lr;
    
    % set up timing
    t_sample = 0.01 ;
    T = unique([0:t_sample:t_f,t_f]);
    N_t = length(T) ;
    
    % get inputs for desired trajectories
    w_slope = -2*(t_f*w_0-psi_end)/t_f^2;
    
    w_traj = w_0+w_slope*T;
    
    v_traj = v_des*ones(1,N_t) ;
    
    if v_des~=0
        wheelangle_traj = atan(l*w_traj./v_traj);
    else
        wheelangle_traj = zeros(1,N_t);
    end
    
    % compute desired trajectory
    z0 = zeros(3,1) ;
    k = [w_0;psi_end;v_des];
    
    [~,Z] = ode45(@(t,z) rover_trajectory_producing_model(t,z,k,t_f),T,z0) ;

    % append velocity and wheelangle to (x,y,h) trajectory to make it a full-state
    % trajectory for the rover
    Z = [Z' ; v_traj;wheelangle_traj] ;
    
    % compute inputs for robot (desired velocity and wheel angle)
    U = [v_traj;wheelangle_traj] ;
    
end