function [T,U,Z] = make_rover_braking_trajectory(t_plan,t_f,t_stop,w_0,psi_end,v_des)
% [T,U,Z] = make_rover_braking_trajectory(t_plan,t_f,t_stop,w_des,v_des)
%
% Create a Dubins path with braking as a full-state trajectory for the
% TurtleBot.
%
% The inputs are:
%   t_plan   planning timeout
%   t_f      planning time horizon
%   t_stop   duration required for robot to come to a stop
%   w_0      desired initial yaw rate
%   psi_end  desired heading at t_f
%   v_des    desired speed
%
% The outputs are:
%   T        timing for desired trajectory as a 1-by-N array
%   U        desired input (velociry and wheel angle) as 2-by-N array
%   Z        desired trajectory (x,y,h,v,delta) as a 5-by-N array
%
% Author: Sean Vaskov
% Created: 6 March 2020


    [T,U,Z] = make_rover_desired_trajectory(t_f,w_0,psi_end,v_des) ;
    [T,U,Z] = convert_rover_desired_to_braking_traj(t_plan,t_stop,T,U,Z) ;
end