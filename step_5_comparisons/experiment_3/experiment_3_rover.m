function experiment_3_rover(index_list,save_flag,plot_flag)

%% description
% This script runs RTD for the rover with minimal sensor horizon and
% realtime planning constraints
%
% NOTE, to run this script, you should be in the following directory:
%   IJRR_bridging_the_gap/step_5_comparisons/experiment_3/rover_data
%
% Author: Sean Vaskov
% Created: 26 Mar 2020
%
%% user parameters

%sensor radius
sensor_radius = 3.0;

% planner

buffer_rtd =  0.01;
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;

lookahead_distance = 4;

reachable_set_poly = [-0.3, 0.2,   1.75,  3.5,  3.5 ,  1.75, 0.2,-0.3,-0.3;...
                      -0.2,-0.3,-2,-2,2 , 2,0.3,0.2,-0.2];
                  
FRS_directory = '~/MATLAB/IJRR_bridging_the_gap/step_3_FRS_computation/data/rover_reconstructed';


% simulation
verbose_level = 2 ;
max_sim_iterations = 120 ;
max_sim_time = t_plan*max_sim_iterations*1.25 ;
plot_while_running = plot_flag;
plot_frs_contour = plot_flag;
plot_hlp = plot_flag ;
allow_replan_errors = false;

% file i/o
summary_filename_header = 'rover_experiment_3_summary' ;

%% automated from here
% create agent
% create roveragent
RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
A = RoverAWD('LLC',RoverLLC,'max_speed',3,'sensor_radius',sensor_radius) ;

% create planners

P = rover_RTD_planner('FRS_directory',FRS_directory,'HLP',lane_HLP,'timeout',t_plan,'t_plan',t_plan,'t_move',t_move,...
    'buffer',buffer_rtd,'lookahead_distance',lookahead_distance,'plot_FRS_flag',plot_frs_contour,'plot_HLP_flag',plot_hlp,'name','RTD','filtering_poly',reachable_set_poly);


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

end