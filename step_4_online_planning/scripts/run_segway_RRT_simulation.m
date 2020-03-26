%% description
% This script runs a simulation with the segway in the simulator
% framework, using RRT to plan online.
%
% Author: Shreyas Kousik
% Created: 24 Mar 2020
% Updated: 25 Mar 2020
%
%% user parameters
% world
obstacle_size_bounds = [0.3, 0.3] ; % side length [min, max]
N_obstacles = 10 ;
bounds = [-4,5,-2.5,2.5] ;
goal_radius = 0.5 ;

% planner
additional_buffer = 0.05 ; % m (this is added to the agent footprint)
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;
plot_HLP_flag = true ;
plot_waypoints_flag = true ;

% simulation
verbose_level = 10 ;
max_sim_time = 1000 ;
max_sim_iterations = 1000 ;

% plotting
plot_while_simulating_flag = true ;
animate_after_simulating_flag = false ;

%% automated from here
A = segway_agent() ;

buffer = A.footprint + additional_buffer ;

P = segway_RRT_planner('verbose',verbose_level,'buffer',buffer,...
    't_plan',t_plan,'t_move',t_move,...
    'plot_HLP_flag',plot_HLP_flag,...
    'plot_waypoints_flag',plot_waypoints_flag) ;

W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',buffer,...
    'verbose',verbose_level,'goal_radius',goal_radius,...
    'obstacle_size_bounds',obstacle_size_bounds) ;

S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
    'max_sim_time',max_sim_time,...
    'max_sim_iterations',max_sim_iterations,...
    'plot_while_running',plot_while_simulating_flag) ;

%% run simulation
S.run() ;

%% animate simulation
if animate_after_simulating_flag
    S.animate()
end
