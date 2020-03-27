clear all

%% description
% This script runs RRT and NMPC planners with different buffer sizes on
% randomly-generated worlds, to determine which buffer size is best
%
% NOTE, to run this script, you should be in the following directory:
%   IJRR_bridging_the_gap/step_5_comparisons/experiment_1/rover_data
%
% Author: Sean Vaskov
% Created: 26 Mar 2020
%
%% user parameters
% worlds
world_start_index = 1 ;
world_end_index = 1000 ;

% planner
additional_buffers = [0 0.05 0.10]; % m
t_plan = 10 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;
T_min = 1.5; %enough time to plan braking trajectory from 2 m/s
T_max = 5;
desired_speed = 2;
speed_weight = 1;
lateral_weight = 1;
lateral_weight_rrt = 10;
longitudinal_weight = 1;
steering_weight = 1;
steering_weight_rrt = 10;
lookahead_distance = 4;
max_heading = 0.6;

run ~/MATLAB/GPOPS-II/gpopsMatlabPathSetup.m

% simulation
run_RRT_planner_flag = true ;
run_NMPC_planner_flag = true ;
verbose_level = 2 ;
max_sim_time = 50 ;
max_sim_iterations = 100 ;
plot_while_running = false ;

% file i/o
save_flag = true ;
summary_filename_header = 'rover_experiment_1_summary' ;

%% automated from here
% create agent
% create roveragent
RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
A = RoverAWD('LLC',RoverLLC,'max_speed',3) ;

% create planners
P = {} ;
p_idx = 1 ;
for additional_buffer = additional_buffers
    
    buffer = additional_buffer ;
    
    if run_RRT_planner_flag
        P{p_idx} = rover_RRT_planner('rrt_dynamics',@rover_rrt_dynamics,'buffer',buffer,...
                                 't_plan',t_plan,'timeout',t_plan,'t_move',t_move,'HLP',lane_HLP,...
                                  'speed_weight',speed_weight,'steering_weight',steering_weight_rrt,...
                                  'lateral_weight',lateral_weight_rrt, 'longitudinal_weight',longitudinal_weight,'T_min',T_min,'T_max',T_max,...
                                  'max_distance',desired_speed*T_max,'desired_speed',desired_speed,'lookahead_distance',lookahead_distance,...
                                  'max_heading',max_heading);
        p_idx = p_idx + 1 ;
    end
    
    if run_NMPC_planner_flag
        P{p_idx} =  rover_GPOPS_planner('HLP',lane_HLP,'timeout',t_plan,'t_plan',t_plan,'t_move',t_move,'T_min',T_min,'T_max',T_max,...
                        'speed_weight',speed_weight,'steering_weight',steering_weight,'lateral_weight',lateral_weight,...
                         'longitudinal_weight',longitudinal_weight,'desired_speed',desired_speed,'lookahead_distance',lookahead_distance,...
                         'max_heading',0.6,'buffer',buffer);
        
        p_idx = p_idx + 1 ;
    end
end

load('rover_simulation_worlds.mat')

%% run simulations
for w_idx = world_start_index:world_end_index
    % get the current world
    W = W_all{w_idx} ;
    
    % adjust the world properties
    W.verbose = verbose_level ;
    W.buffer = 0;
    % make a simulator for this world and all of the planners
    S = simulator(A,W,P,'allow_replan_errors',false,'verbose',verbose_level,...
        'max_sim_time',max_sim_time,...
        'max_sim_iterations',max_sim_iterations,...
        'plot_while_running',plot_while_running) ;
    
    % run the simulator
    summary = S.run() ;
    
    % save the summary
    if save_flag
        summary_save_filename = [summary_filename_header,...
            '_world_',num2str(w_idx,'%04d'),'.mat'] ;
        save(summary_save_filename, 'summary')
    end
end