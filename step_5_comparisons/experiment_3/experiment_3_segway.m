function experiment_3_segway(world_start_index,world_end_index,...
    save_flag,plot_while_running_flag)
% experiment_3_segway(world_start_index,world_end_index,...
%   save_flag,plot_while_running_flag)
%
% This function runs the RTD planner with a limited sensor horizon on the
% randomly-generated worlds in segway_simulation_worlds.mat.
%
% NOTE, to run this script, you should be in the following directory:
%   IJRR_bridging_the_gap/step_5_comparisons/experiment_3/segway_data/
%
% Author: Shreyas Kousik
% Created: 27 Mar 2020
% Updated: -
%
%% experiment parameters
    % agent
    sensor_radius = 1.875 ;

    % planner
    buffer = 0.001 ; % m (this is added to the agent footprint)
    t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
    t_move = 0.5 ;
    plot_HLP_flag = true ;
    plot_waypoints_flag = true ;
    plot_FRS_flag = true ;

    % simulation
    verbose_level = 4 ;
    max_sim_time = 300 ;
    max_sim_iterations = 100 ;
    if nargin < 3
        plot_while_running_flag = false ;
    end

    % file i/o
    if nargin < 3
        save_flag = true ;
    end
    summary_filename_header = 'segway_experiment_3_summary' ;

    %% experiment setup
    % create agent
    A = segway_agent('sensor_radius',sensor_radius) ;

    % create planners
    P = segway_RTD_planner('verbose',verbose_level,'buffer',buffer,...
        't_plan',t_plan,'t_move',t_move,...
        'plot_HLP_flag',plot_HLP_flag,...
        'plot_waypoints_flag',plot_waypoints_flag,...
        'plot_FRS_flag',plot_FRS_flag) ;

    load('segway_simulation_worlds.mat')

    %% run simulations
    for w_idx = world_start_index:world_end_index
        % get the current world
        W = W_all{w_idx} ;

        % adjust the world properties
        W.buffer = A.footprint + buffer ;
        W.verbose = verbose_level ;

        % make a simulator for this world and all of the planners
        S = simulator(A,W,P,...
            'allow_replan_errors',false,...
            'verbose',verbose_level,...
            'max_sim_time',max_sim_time,...
            'max_sim_iterations',max_sim_iterations,...
            'plot_while_running',plot_while_running_flag) ;

        % run the simulator
        summary = S.run() ;

        % save the summary
        if save_flag
            summary_save_filename = [summary_filename_header,...
                '_world_',num2str(w_idx,'%04d'),'.mat'] ;
            save(summary_save_filename, 'summary')
        end
    end
end
