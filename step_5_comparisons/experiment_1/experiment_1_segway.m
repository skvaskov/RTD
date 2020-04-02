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
% Updated: 2 Apr 2020
%
%% experiment parameters
    % agent
    sensor_radius = 100 ;

    % planner
    t_move = 0.5 ;
    t_plan = 10 ; % if t_plan = t_move, then real time planning is enforced
    buffer_sizes = [0.40 0.45 0.50 0.65] ; % m (this is added to the agent footprint)
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
    for buffer = buffer_sizes

        if run_RRT_planner_flag
            n = ['Segway RRT Planner, Experiment 1, Buffer = ',num2str(buffer,'%0.2f'),' m'] ;
            
            P{p_idx} = segway_RRT_planner('verbose',verbose_level,'buffer',buffer,...
                't_plan',t_plan,'t_move',t_move,...
                'plot_HLP_flag',plot_HLP_flag,...
                'plot_waypoints_flag',plot_waypoints_flag,...
                'name',n) ;

            p_idx = p_idx + 1 ;
        end

        if run_NMPC_planner_flag
            n = ['Segway NMPC Planner, Experiment 1, Buffer = ',num2str(buffer,'%0.2f'),' m'] ;
            
            P{p_idx} = segway_NMPC_planner('verbose',verbose_level,'buffer',buffer,...
                't_plan',t_plan,'t_move',t_move,...
                'plot_HLP_flag',plot_HLP_flag,...
                'plot_waypoints_flag',plot_waypoints_flag,...
                'name',n) ;

            p_idx = p_idx + 1 ;
        end
    end

    load('segway_simulation_worlds.mat')

%% run experiment
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
