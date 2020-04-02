function experiment_2_segway(world_start_index,world_end_index,...
    save_flag,plot_while_running_flag)
% experiment_2_segway(world_start_index,world_end_index,...
%   save_flag,plot_while_running_flag)
%
% This function RTD, RRT, and NMPC planners on the randomly-generated
% worlds in segway_simulation_worlds.mat, to compare them on the axes of
% number of goals reached and number of crashes
%
% NOTE, to run this function, you should be in the following directory:
%   IJRR_bridging_the_gap/step_5_comparisons/experiment_2/segway_data/
%
% Author: Shreyas Kousik
% Created: 27 Mar 2020
% Updated: 2 Apr 2020
%
%% experiment parameters
    % agent
    sensor_radius = 100 ;

    % planners
    t_move = 0.5 ;
    t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
    RTD_buffer = 0.001 ;
    RRT_buffer = 0.45 ;
    NMPC_buffer = 0.45 ;
    plot_HLP_flag = true ;
    plot_waypoints_flag = true ;

    % simulation
    run_RTD_planner_flag = true ;
    run_RRT_planner_flag = true ;
    run_NMPC_planner_flag = true ;
    verbose_level = 4 ;
    max_sim_time = 300 ;
    max_sim_iterations = 100 ;
    if nargin < 4
        plot_while_running_flag = false ;
    end

    % file i/o
    if nargin < 3
        save_flag = true ;
    end
    summary_filename_header = 'segway_experiment_2_summary' ;

%% experiment setup
    % create agent
    A = segway_agent('sensor_radius',sensor_radius) ;
    
    % create planners
    P = {} ;
    p_idx = 1 ;
    
    if run_RTD_planner_flag
        n = 'Segway RTD Planner, Experiment 2' ;
        
        P{p_idx} = segway_RTD_planner('verbose',verbose_level,...
            'buffer',RTD_buffer,...
            't_plan',t_plan,'t_move',t_move,...
            'plot_HLP_flag',plot_HLP_flag,...
            'plot_waypoints_flag',plot_waypoints_flag,...
            'name',n) ;
        
        p_idx = p_idx + 1 ;
    end
    
    if run_RRT_planner_flag
        n = 'Segway RRT Planner, Experiment 2' ;
        
        P{p_idx} = segway_RRT_planner('verbose',verbose_level,...
            'buffer',RRT_buffer,...
            't_plan',t_plan,'t_move',t_move,...
            'plot_HLP_flag',plot_HLP_flag,...
            'plot_waypoints_flag',plot_waypoints_flag,...
            'name',n) ;
        
        p_idx = p_idx + 1 ;
    end
    
    if run_NMPC_planner_flag
        n = 'Segway NMPC Planner, Experiment 2' ;
        
        P{p_idx} = segway_NMPC_planner('verbose',verbose_level,...
            'buffer',NMPC_buffer,...
            't_plan',t_plan,'t_move',t_move,...
            'plot_HLP_flag',plot_HLP_flag,...
            'plot_waypoints_flag',plot_waypoints_flag,...
            'name',n) ;
        
        p_idx = p_idx + 1 ;
    end
    
    % load worlds
    load('segway_simulation_worlds.mat')

%% run experiment
    for w_idx = world_start_index:world_end_index
        % get the current world
        W = W_all{w_idx} ;

        % adjust the world properties
        W.buffer = max([RTD_buffer RRT_buffer NMPC_buffer]) ;
        W.verbose = verbose_level ;

        % make a simulator for this world and all of the planners
        S = simulator(A,W,P,'allow_replan_errors',false,'verbose',verbose_level,...
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
