function experiment_1_segway(world_start_index,world_end_index,...
    save_flag,plot_while_running_flag)
% experiment_1_segway(world_start_index,world_end_index,...
%   save_flag,plot_while_running_flag)
%
% This function runs RRT and NMPC planners with different buffer sizes on
% randomly-generated worlds in segway_simulation_worlds.mat, to determine
% which buffer size is best at balancing the tradeoff between safety and
% liveness.
%
% NOTE, to run this function, you should be in the following directory:
%   IJRR_bridging_the_gap/step_5_comparisons/experiment_1/segway_data/
%
% Author: Shreyas Kousik
% Created: 24 Mar 2020
% Updated: 27 Mar 2020
%
%% experiment parameters
    % agent
    sensor_radius = 100 ;

    % planner
    additional_buffers = [0.0, 0.05, 0.10, 0.15] ; % m (this is added to the agent footprint)
    t_plan_RRT = 0.5 ; % if t_plan = t_move, then real time planning is enforced
    t_plan_NMPC = 10 ;
    t_move = 0.5 ;
    plot_HLP_flag = true ;
    plot_waypoints_flag = true ;

    % simulation
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
    summary_filename_header = 'segway_experiment_1_summary' ;

    %% experiment setup
    % create agent
    A = segway_agent('sensor_radius',sensor_radius) ;

    % create planners
    P = {} ;
    p_idx = 1 ;
    for additional_buffer = additional_buffers

        buffer = A.footprint + additional_buffer ;

        if run_RRT_planner_flag
            P{p_idx} = segway_RRT_planner('verbose',verbose_level,'buffer',buffer,...
                't_plan',t_plan_RRT,'t_move',t_move,...
                'plot_HLP_flag',plot_HLP_flag,...
                'plot_waypoints_flag',plot_waypoints_flag) ;

            p_idx = p_idx + 1 ;
        end

        if run_NMPC_planner_flag
            P{p_idx} = segway_NMPC_planner('verbose',verbose_level,'buffer',buffer,...
                't_plan',t_plan_NMPC,'t_move',t_move,...
                'plot_HLP_flag',plot_HLP_flag,...
                'plot_waypoints_flag',plot_waypoints_flag) ;

            p_idx = p_idx + 1 ;
        end
    end

    load('segway_simulation_worlds.mat')

    %% run simulations
    for w_idx = world_start_index:world_end_index
        % get the current world
        W = W_all{w_idx} ;

        % adjust the world properties
        W.buffer = max(A.footprint + additional_buffers) ;
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
