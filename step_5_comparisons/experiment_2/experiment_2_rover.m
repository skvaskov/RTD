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

%sensor radius
sensor_radius = 5;

% worlds
% index_list = [1,19,27,32,38,42,54,57,58,67,70,71,84,89,90,96,97,101,104,105,109,110,140,142,144,146,149,154,160,165,170,171,176,177,179,180,188,191,192,193,194,202,204,205,207,209,213,216,232,234,235,239,241,243,255,265,267,268,280,285,286,291,297,302,307,308,310,322,325,327,328,329,331,332,341,344,349,351,356,357,359,365,369,371,372,373,379,387,393,394,396,399,402,403,404,406,411,414,419,425,428,441,446,455,456,460,464,467,468,470,471,481,487,490,495,500,501,514,525,530,533,535,543,544,551,553,554,567,579,589,594,598,601,608,610,612,613,624,625,631,633,635,647,650,652,659,662,674,675,676,688,689,694,699,700,702,707,709,710,715,716,718,723,725,731,732,740,743,744,745,754,756,757,758,763,767,771,772,773,774,781,787,795,805,807,810,814,816,823,826,831,838,842,846,854,858,863,864,867,871,872,875,878,891,898,913,914,915,918,931,932,937,940,941,946,948,950,962,964,979,983,988];

index_list = 1;

% planner
buffer_rrt =  0.10; % m
buffer_nmpc = 0.05;
buffer_rtd =  0.01;
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
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
lookahead_distance = 5;
max_heading = 0.6;
reachable_set_poly = [-0.3, 0.2,   1.75,  3.5,  3.5 ,  1.75, 0.2,-0.3,-0.3;...
                      -0.2,-0.3,-2,-2,2 , 2,0.3,0.2,-0.2];
FRS_directory = '/Users/seanvaskov/MATLAB/IJRR_bridging_the_gap/step_3_FRS_computation/data/rover_reconstructed';

run ~/MATLAB/GPOPS-II/gpopsMatlabPathSetup.m

% simulation
run_RRT_planner_flag = false ;
run_NMPC_planner_flag = false;
run_RTD_planner_flag = true ;

verbose_level = 2 ;
max_sim_iterations = 120 ;
max_sim_time = t_plan*max_sim_iterations*1.25 ;
plot_while_running = true;
plot_frs_contour = true;
plot_hlp = true;
allow_replan_errors = false;

% file i/o
save_flag = false;
summary_filename_header = 'rover_experiment_2_summary' ;

%% automated from here
% create agent
% create roveragent
RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
A = RoverAWD('LLC',RoverLLC,'max_speed',3,'sensor_radius',sensor_radius) ;

% create planners
P = {} ;
p_idx = 1 ;

    
    
if run_RRT_planner_flag
    P{p_idx} = rover_RRT_planner('rrt_dynamics',@rover_rrt_dynamics,'buffer',buffer_rrt,...
        't_plan',t_plan,'timeout',t_plan,'t_move',t_move,'HLP',lane_HLP,...
        'speed_weight',speed_weight,'steering_weight',steering_weight_rrt,...
        'lateral_weight',lateral_weight_rrt, 'longitudinal_weight',longitudinal_weight,'T_min',T_min,'T_max',T_max,...
        'max_distance',desired_speed*T_max,'desired_speed',desired_speed,'lookahead_distance',lookahead_distance,...
        'max_heading',max_heading,'name','RRT','plot_HLP_flag',plot_hlp);
    p_idx = p_idx + 1 ;
end

if run_NMPC_planner_flag
    P{p_idx} =  rover_GPOPS_planner('HLP',lane_HLP,'timeout',t_plan,'t_plan',t_plan,'t_move',t_move,'T_min',T_min,'T_max',T_max,...
        'speed_weight',speed_weight,'steering_weight',steering_weight,'lateral_weight',lateral_weight,...
        'longitudinal_weight',longitudinal_weight,'desired_speed',desired_speed,'lookahead_distance',lookahead_distance,...
        'max_heading',0.6,'buffer',buffer_nmpc,'name','NMPC','plot_HLP_flag',plot_hlp);
    
    p_idx = p_idx + 1 ;
end

if run_RTD_planner_flag
    P{p_idx} = rover_RTD_planner('FRS_directory',FRS_directory,'HLP',lane_HLP,'timeout',t_plan,'t_plan',t_plan,'t_move',t_move,...
        'buffer',buffer_rtd,'lookahead_distance',lookahead_distance,'plot_FRS_flag',plot_frs_contour,'plot_HLP_flag',plot_hlp,'name','RTD','filtering_poly',reachable_set_poly);
    
end

load('rover_simulation_worlds.mat')

%% run simulations
for w_idx = index_list
    % get the current world
    W = W_all{w_idx} ;
    
    % adjust the world properties
    W.verbose = verbose_level ;
    W.buffer = 0;
    % make a simulator for this world and all of the planners
    S = simulator(A,W,P,'allow_replan_errors',allow_replan_errors,'verbose',verbose_level,...
        'max_sim_time',max_sim_time,...
        'max_sim_iterations',max_sim_iterations,...
        'plot_while_running',plot_while_running) ;
    
    % run the simulator
    summary = S.run() ;

    % save the summary
    if save_flag
        summary_save_filename = [summary_filename_header,...
            '_world_',num2str(w_idx,'%04d'),'.mat'] ;
        save(summary_save_filename, 'summary','w_idx')
    end
end