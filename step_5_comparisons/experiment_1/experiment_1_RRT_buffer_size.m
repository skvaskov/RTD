%% description
% This script run the RRT planner with several different buffer sizes on
% randomly-generated worlds, to determine which buffer size is best
%
% NOTE, to run this script, you should be in the following directory:
%   IJRR_bridging_the_gap/step_5_comparisons/experiment_1/segway_data
%
% Author: Shreyas Kousik
% Created: 24 Mar 2020
% Updated: 25 Mar 2020
%
%% user parameters
% worlds
N_worlds = 500 ;
N_obstacle_bounds = [5,10] ;
bounds = [-4,5,-2.5,2.5] ;
goal_radius = 0.5 ;

% planner
additional_buffers = [0.0, 0.05, 0.10, 0.15] ; % m (this is added to the agent footprint)
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;
plot_HLP_flag = true ;
plot_waypoints_flag = true ;

% simulation
verbose_level = 2 ;
max_sim_time = 50 ;
max_sim_iterations = 100 ;
plot_while_running = true ;

% file i/o
load_worlds_flag = false ;
save_flag = true ;
worlds_save_filename = 'segway_RRT_experiment_1_worlds.mat' ;
summary_save_filename_header = 'segway_RRT_experiment_1_summary' ;

%% automated from here
% create agent
A = segway_agent() ;

% create planners
P = {} ;
p_idx = 1 ;
for buffer = additional_buffers
    HLP = RRT_star_HLP('grow_tree_mode','new',...
        'plot_while_growing_tree_flag',false,...
        'new_node_max_distance_from_agent',A.sensor_radius - A.footprint) ;
    
    P{p_idx} = segway_RRT_planner('verbose',verbose_level,'buffer',A.footprint + buffer,...
        'HLP',HLP,'t_plan',t_plan,'t_move',t_move,...
        'plot_HLP_flag',plot_HLP_flag,...
        'plot_waypoints_flag',plot_waypoints_flag) ;
    p_idx = p_idx + 1 ;
end

% create worlds
if load_worlds_flag
    load(worlds_save_filename)
else
    world_buffer = max(A.footprint + additional_buffers) ;
    W_all = {} ;
    for w_idx = 1:N_worlds
        N_obstacles = rand_int(N_obstacle_bounds(1),N_obstacle_bounds(2)) ;
        
        W_all{w_idx} = static_box_world('bounds',bounds,...
            'N_obstacles',N_obstacles,'buffer',world_buffer,...
            'verbose',verbose_level,'goal_radius',goal_radius,...
            'obstacle_size_bounds',[0.3, 0.3]) ;
    end
end

if save_flag
    save(worlds_save_filename, 'W_all')
end

%% run simulations
for w_idx = 1:N_worlds
    S = simulator(A,W_all{w_idx},P,'allow_replan_errors',false,'verbose',verbose_level,...
        'max_sim_time',max_sim_time,...
        'max_sim_iterations',max_sim_iterations,...
        'plot_while_running',plot_while_running) ;
    
    summary = S.run() ;
    
    if save_flag
        summary_save_filename = [summary_save_filename_header,'_world_',num2str(w_idx,'%04d'),'.mat'] ;
        save(summary_save_filename, 'summary')
    end
end