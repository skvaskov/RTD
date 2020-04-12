classdef simulator < handle
% Class: simulator
%
% S = simulator(agents, worlds, planners, varargin)
%
% Authors: Shreyas Kousik, Sean Vaskov, and Hannah Larson
% Created: a long, long time ago, on a laptop far, far away
% Updated: 8 Mar 2020

%% properties
    properties (Access = public)
        % basic properties
        agents = {agent()} ;
        worlds = {world()} ;
        planners = {planner()} ;
        
        N_agents = 1 ;
        N_worlds = 1 ;
        N_planners = 1 ;
        
        % user-friendly properties
        verbose = 1 ;

        % simulation
        max_sim_time = 100 ; % s per planner
        max_sim_iterations = 100 ; % per planner
        stop_count = 0 ;
        stop_threshold = 20 ;
        stop_sim_when_crashed = true ;
        allow_replan_errors = false ;
        save_planner_info = false ;
        manual_iteration = false ;
        collision_check_full_traj_after_sim_flag = false ;
        simulation_summary = [] ;
        

        % plotting
        figure_number = 1 ;
        figure_handle = [] ;
        plot_while_running = true ;
        plotting_pause_time = 0.1 ; % s
        plot_order = 'WAP' ;
        save_gif = false ;
        save_gif_filename = 'simulator_gif_output.gif' ;
        save_gif_delay_time = 0.1 ;
        start_gif = true ;
        manually_resize_gif = true ;
        animation_time_discretization = 0.1 ; % s
        animation_linewidths = 1 ;
        clear_plot_before_animating_flag = false ; 
        set_plot_linewidths_flag = false ;
        set_axes_while_animating_flag = false ;
    end

%% methods
    methods
    %% constructor
        function S = simulator(agents, worlds, planners, varargin)
            % Constructor function: simulator
            %
            % Usage: S = simulator(agents, world, planners, varargin)
            %
            % This constructor takes in agents, which obey some physical
            % dynamics; world objects, which contains obstacles and goals;
            % and planners for the agent to perform receding-horizon traj.
            % planning in the worlds.

            % parse inputs
            S = parse_args(S,varargin{:}) ;
            
            % get number of agents, worlds, and planners
            S.N_agents = length(agents) ;
            S.N_worlds = length(worlds) ;
            S.N_planners = length(planners) ;
            
            % error if N_agents > 1 and N_agents ~= N_planners
            if S.N_agents > 1 && S.N_agents ~= S.N_planners
                error('Please provide either one agent, or one agent per planner')
            end

            % if the agent, world, or planner are alone, wrap them in cells
            if S.N_agents == 1 && ~iscell(agents)
                agents = {agents} ;
            end
            
            if S.N_worlds == 1 && ~iscell(worlds)
                worlds = {worlds} ;
            end

            if S.N_planners == 1 && ~iscell(planners)
                planners = {planners} ;
            end

            % wrap up construction
            S.agents = agents ;
            S.worlds = worlds ;
            S.planners = planners ;
        end

    %% run simulation
        function summary = run(S,planner_indices)
            % Method: run
            %
            % This function simulates the agent in the provided world as it
            % attempts to reach the goal from its provided start position.
            
            S.vdisp('Running simulation')

            % get simulation info
            t_max = S.max_sim_time ;
            iter_max = S.max_sim_iterations ;
            plot_in_loop_flag = S.plot_while_running ;

            % get world and planner indices
            world_indices = 1:length(S.worlds) ;
            
            if nargin < 2
                planner_indices = 1:length(S.planners) ;
            end

            % set up summaries
            LW = length(world_indices) ;
            summary = cell(1,LW) ;

        %% world loop
            for widx = world_indices
                W = S.get_world(widx) ;

                % set up summary objects
                L = length(planner_indices) ;
                agent_name_cell = cell(1,L) ;
                planner_name_cell = cell(1,L) ;
                planner_info_cell = cell(1,L) ;
                agent_info_cell = cell(1,L) ;
                trajectory_cell = cell(1,L) ;
                total_real_time_cell = cell(1,L) ;
                total_iterations_cell = cell(1,L) ;
                planning_times_cell = cell(1,L) ;
                collision_check_cell = cell(1,L) ;
                goal_check_cell = cell(1,L) ;
                stop_check_cell = cell(1,L) ;
                total_simulated_time_cell = cell(1,L) ;
                control_input_cell = cell(1,L) ;
                control_input_time_cell = cell(1,L) ;
                t_plan_cell = cell(1,L) ;
                t_move_cell = cell(1,L) ;
                obstacles_cell = cell(1,L) ;
                
            %% planner loop
                for pidx = planner_indices
                    S.vdisp(['Planner ',num2str(pidx)])

                    % get agent and planner
                    A = S.get_agent(pidx) ;
                    P = S.get_planner(pidx) ;

                    % get agent and world ready
                    W.reset() ;
                    A.reset(W.start) ; 

                    % get planner ready
                    agent_info = A.get_agent_info() ;
                    world_info = W.get_world_info(agent_info,P) ;
                    P.setup(agent_info,world_info) ;

                    % check to make sure gif start is ready
                    if S.save_gif
                        S.start_gif = true ;
                    end

                    % initialize plot
                    if plot_in_loop_flag
                        S.plot(widx,pidx)
                    end

                    % preallocate for storing planning time spent
                    planning_time_vec = nan(1,iter_max) ;

                    % reset the stop counter
                    S.stop_count = 0 ;
                    stop_check_vec = false(1,iter_max) ;
                    
                    % reset the crash and goal checks just in case
                    collision_check = false ;
                    goal_check = false ;

                    % start timing
                    icur = 1 ;
                    runtime = tic ;
                    tstart = runtime ;
                    tcur = toc(tstart);

                %% simulation loop
                    while icur < (iter_max+1) && tcur < t_max
                        S.vdisp('--------------------------------',3,false)
                        S.vdisp(['ITERATION ',num2str(icur),' (t = ',num2str(A.time(end),'%0.2f'),')'],2,false)

                    %% get agent info
                        agent_info = A.get_agent_info() ;

                    %% get world info
                        % given the current state of the agent, query the world
                        % to get the surrounding obstacles
                        world_info = W.get_world_info(agent_info,P) ;

                    %% replan
                        % given the current state and obstacles, query the
                        % current planner to get a control input
                        t_plan_spent = tic ;
                        if S.allow_replan_errors
                            [T_nom,U_nom,Z_nom] = P.replan(agent_info,world_info) ;
                        else
                            try
                                [T_nom,U_nom,Z_nom] = P.replan(agent_info,world_info) ;
                            catch
                                S.vdisp(['Planner ',num2str(pidx),' errored while ',...
                                         'replanning!'])
                                T_nom = [] ; U_nom = [] ; Z_nom = [] ;
                            end
                        end
                        t_plan_spent = toc(t_plan_spent) ;
                        planning_time_vec(icur) = t_plan_spent ;
                        S.vdisp(['Planning time: ',num2str(t_plan_spent),' s'],4)

                    %% move agent
                        % update the agent using the current control input, so
                        % either stop if no control was returned, or move the
                        % agent if a valid input and time vector were returned
                        if size(T_nom,2) < 2 || size(U_nom,2) < 2 || T_nom(end) == 0
                            S.vdisp('Stopping!',2)
                            A.stop(P.t_move) ;

                            stop_check_vec(icur) = true ;

                            % give planner a chance to recover from a stop
                            S.stop_count = S.stop_count + 1 ;
                            if S.stop_count > S.stop_threshold
                                break
                            end
                        else
                            S.stop_count = 0 ;

                            if ~isempty(P.t_move)
                                if P.t_move > T_nom(end)
                                    S.vdisp(['The provided time vector for the ',...
                                        'agent input is shorter than the amount of ',...
                                        'time the agent must move at each ',...
                                        'planning iteration. The agent will only ',...
                                        'be moved for the duration of the ',...
                                        'provided time vector.'],3)

                                    t_move = T_nom(end) ;
                                else
                                    t_move = P.t_move ;
                                end
                            else
                                error(['Planner ',num2str(pidx),...
                                       '''s t_move property is empty!'])
                            end

                            A.move(t_move,T_nom,U_nom,Z_nom) ;
                        end

                    %% Note (22 July 2019)
                    % Dynamic obstacles are treated as follows:
                    %   1) W.get_world_info should return a prediction
                    %   2) the agent is moved according to the prediction
                    %   3) the world moves the obstacles (according to the
                    %      agent's movement data if needed) and then checks
                    %      for collisions in W.collision_check

                    %% crash and goal check
                        % check if the agent is near the desired goal or if it
                        % crashed
                        S.vdisp('Checking if agent reached goal or crashed...',3)
                        agent_info = A.get_agent_info() ;
                        goal_check = W.goal_check(agent_info) ;
                        collision_check = W.collision_check(agent_info,false) ;
                        
                        if isa(A,'multi_link_agent')
                            S.vdisp('Checking for self-intersection.',2)
                            collision_check = collision_check || A.self_intersection_flag ;
                        end

                        if collision_check && S.stop_sim_when_crashed
                            S.vdisp('Crashed!',2) ;
                            break
                        end

                        if goal_check
                            S.vdisp('Reached goal!',2) ;
                            break
                        end

                        % plotting and animation
                        if plot_in_loop_flag
                            S.plot(widx,pidx)
                            if S.save_gif
                                error('Shreyas this is unfinished!')
                            else
                                pause(S.plotting_pause_time) ;
                            end
                        end

                        % pause for user if needed
                        if S.manual_iteration
                            user_pause = tic ;
                            S.vdisp('Pausing for user. Press any key to continue.',2)
                            pause
                            user_pause = toc(user_pause) ;
                        else
                            user_pause = 0 ;
                        end

                        % iterate and increment time
                        S.vdisp(['END ITERATION ',num2str(icur)],4,false)
                        icur = icur + 1 ;
                        tcur = toc(tstart) - user_pause ;
                    end
                    runtime = toc(runtime) ;
                    S.vdisp(['Planning time spent: ',num2str(runtime)],5)

                    % plot the last portion of the agent's trajectory after the
                    % simulation ends
                    if plot_in_loop_flag
                        S.plot(widx,pidx)
                    end

                    S.vdisp(['Planner ',num2str(pidx), ' simulation complete!'])

                %% create summary (for the current planner)
                    % get results at end of simulation
                    S.vdisp('Compiling summary',3)
                    Z = A.state ;
                    T_nom = A.time ;
                    U_nom = A.input ;
                    TU = A.input_time ;
                    agent_info = A.get_agent_info() ;
                    
                    if S.collision_check_full_traj_after_sim_flag
                        S.vdisp('Running final collision check.',4)
                        collision_check = W.collision_check(agent_info) ;
                    end
                    
                    S.vdisp('Running final goal check',4)
                    goal_check = W.goal_check(agent_info) ;
                    agent_info_cell{pidx} = agent_info ;

                    if S.save_planner_info
                        planner_info_cell{pidx} = S.planners{pidx}.info ;
                    else
                        planner_info_cell{pidx} = 'no info saved' ;
                    end

                    % fill in the results for the current planner
                    agent_name_cell{pidx} = A.name ;
                    planner_name_cell{pidx} = P.name ;
                    trajectory_cell{pidx} = Z ;
                    total_simulated_time_cell{pidx} = T_nom ;
                    control_input_cell{pidx} = U_nom ;
                    control_input_time_cell{pidx} = TU ;
                    total_real_time_cell{pidx} = runtime ;
                    total_iterations_cell{pidx} = icur ;
                    planning_times_cell{pidx} = planning_time_vec ;
                    collision_check_cell{pidx} = collision_check ;
                    goal_check_cell{pidx} = goal_check ;
                    stop_check_cell{pidx} = stop_check_vec ;
                    t_plan_cell{pidx} = P.t_plan ;
                    t_move_cell{pidx} = P.t_move ;
                    obstacles_cell{pidx} = W.obstacles;
                    
                    if goal_check
                        S.vdisp('In final check, agent reached goal!')
                    end
                    
                    if collision_check
                        S.vdisp('In final check, agent crashed!')
                    end
                end
                S.vdisp(['World ',num2str(widx),' complete! Generating summary.'])
                summary{widx} = struct('agent_name',agent_name_cell,...
                                 'planner_name',planner_name_cell,...
                                 'trajectory',trajectory_cell,...
                                 'total_real_time',total_real_time_cell,...
                                 'total_iterations',total_iterations_cell,...
                                 'planning_time',planning_times_cell,...
                                 'collision_check',collision_check_cell,...
                                 'goal_check',goal_check_cell,...
                                 'stop_check',stop_check_cell,...
                                 'total_simulated_time',total_simulated_time_cell,...
                                 'control_input',control_input_cell,...
                                 'control_input_time',control_input_time_cell,...
                                 'agent_info',agent_info_cell,...    
                                 'planner_info',planner_info_cell,...
                                 'obstacles',obstacles_cell,...
                                 'planner_indices',planner_indices,...
                                 'N_obstacles',W.N_obstacles,...
                                 't_plan',t_plan_cell,...
                                 't_move',t_move_cell,...
                                 't_max',t_max,...
                                 'iter_max',iter_max,...
                                 'start',W.start,...
                                 'goal',W.goal,...
                                 'bounds',W.bounds,...
                                 'notes','') ;
                             
            end

            % clean up summary if only one world was run
            if LW == 1
                summary = summary{1} ;
            end
            
            S.simulation_summary = summary ;

            if nargout < 1
                clear summary ;
                S.vdisp('Simulation summary stored in simulator_obj.simulation_summary.',1)
            end
            
            S.vdisp('Simulation complete!')
        end

    %% plotting
    function plot(S,world_index,planner_index)
        S.vdisp('Plotting',3)

        if nargin < 3
            planner_index = 1 ;
            S.vdisp('Plotting planner 1 by default!',1) ;
            if nargin < 2
                world_index = 1 ;
                S.vdisp('Plotting world 1 by default!',1) ;
            end
        end

        fh = figure(S.figure_number) ;
        cla ; hold on ; axis equal ;

        % get agent, world, and planner
        A = S.get_agent(planner_index) ;
        W = S.get_world(world_index) ;
        P = S.get_planner(planner_index) ;

        if ~any(isinf(S.worlds{world_index}.bounds))
            axis(W.bounds)
        end

        for plot_idx = S.plot_order
            switch plot_idx
                case 'A'
                    A.plot()
                case 'W'
                    W.plot()
                case 'P'
                    P.plot()
                otherwise
                    error(['Simulator plot order is broken! Make sure ',...
                          'it is a string containing the characters ',...
                          'A (for agent), W (for world), and P (for ',...
                          'planner), in the order you want them to ',...
                          'plot (WAP is the default).'])
            end
        end

        if S.save_gif            
            if S.start_gif
                % if gif saving is enabled, check that there isn't already a
                % file in the current directory with that name
                dir_content = dir(pwd) ;
                file_name   = {dir_content.name} ;
                check_name = [S.save_gif_filename,'_planner_',num2str(planner_index),'.gif'] ;
                file_check  = any(cellfun(@(x) strcmp(check_name,x),file_name)) ;
                if file_check
                    warning(['The current GIF filename already exists! ',...
                        'change the filename if you do not want to ',...
                        'overwrite your existing file!'])
                end
                if S.manually_resize_gif
                    S.vdisp(['Please resize the figure to the size you ',...
                             'want saved, or hit any key to continue.'])
                    pause
                end
            end

            frame = getframe(fh) ;
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);

            filename = [S.save_gif_filename,'_planner_',num2str(planner_index),'.gif'] ;

            if S.start_gif
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf,...
                        'DelayTime',S.save_gif_delay_time) ;
                S.start_gif = false ;
            else
                imwrite(imind,cm,filename,'gif','WriteMode','append',...
                        'DelayTime',S.save_gif_delay_time) ;
            end
        end
    end
    
    function plot_at_time(S,t,planner_index,world_index)
        % method: plot_at_time(t)
        %
        % Plot the agent, world, and planner at the time t; this calls the
        % plot_at_time method for each of class.
        
        if nargin < 4
            world_index = 1 ;
        end
        
        if nargin < 3
            planner_index = 1 ;
        end
        
        if nargin < 2
            t = 0 ;
        end
        
        % get agent, world, and planner
        A = S.get_agent(planner_index) ;
        W = S.get_world(world_index) ;
        P = S.get_planner(planner_index) ;
        
        % set up hold
        if ~ishold
            hold on
            hold_check = true ;
        else
            hold_check = false ;
        end
        
        for plot_idx = S.plot_order
            switch plot_idx
                case 'A'
                    A.plot_at_time(t)
                case 'W'
                    W.plot_at_time(t)
                case 'P'
                    P.plot_at_time(t)
                otherwise
                    error(['Simulator plot order is broken! Make sure ',...
                        'it is a string containing the characters ',...
                        'A (for agent), W (for world), and P (for ',...
                        'planner), in the order you want them to ',...
                        'plot (WAP is the default).'])
            end
        end
        
        if hold_check
            hold off
        end
    end
    
    function clear_plot_data(S)
        % iterate through each agent, world, and planner, and clear the
        % plot data for each one
        for idx = 1:length(S.agents)
            clear_plot_data(S.agents{idx}) ;
        end
        
        for idx = 1:length(S.planners)
            clear_plot_data(S.planners{idx}) ;
        end
        
        for idx = 1:length(S.worlds)
            clear_plot_data(S.worlds{idx}) ;
        end
    end
    
    %% animate
    function animate(S,planner_index,world_index,save_animation_gif)
    % method: S.animate(planner_index,world_index,save_gif)
    %         S.animate(save_gif)
    %
    % Animate the agent, world, and planner for the duration given by
    % A.time. The time between animated frames is given by the simulator's
    % animation_time_discretization property. The planner and world to
    % animate are given by the planner_index and world_index inputs, so the
    % simulator plots S.worlds{world_index} and S.planners{planner_index}.
    %
    % One can also call this as S.animate(true) to save a GIF for planner 1
    % and world 1 (which is useful if there's only one planner and world
    % currently set up in the simulator).
    
        S.vdisp('Animating the previous simulation',2)
    
        % parse input arguments
        if nargin == 2 && islogical(planner_index)
            save_animation_gif = planner_index ;
            planner_index = 1 ;
            world_index = 1 ;
        else
            if nargin < 4
                save_animation_gif = false ;
                start_animation_gif = false ;
            end

            if nargin < 3
                world_index = 1 ;
            end

            if nargin < 2
                planner_index = 1 ;
            end
        end
        
        if save_animation_gif
            start_animation_gif = true ;
            filename = S.animation_gif_setup() ;
        end
        
        % get agent
        A = S.get_agent(planner_index) ;
        
        % get world
        W = S.get_world(world_index) ;
        
        % get time
        t_vec = A.time(1):S.animation_time_discretization:A.time(end) ;

        % clear the active figure before animating
        if S.clear_plot_before_animating_flag
            clf ;
        end
        
        % set hold
        hold_check = ~ishold ;
        if hold_check
            hold on
        end
        
        for t_idx = t_vec
            % create plot
            S.plot_at_time(t_idx,planner_index,world_index)
            
            if S.set_plot_linewidths_flag
                set_plot_linewidths(S.animation_linewidths) ;
            end
            
            if S.set_axes_while_animating_flag
                axis(W.bounds)
            end
            
            % create gif
            if save_animation_gif
                % get current figure
                fh = get(groot,'CurrentFigure') ;
                frame = getframe(fh) ;
                im = frame2im(frame);
                [imind,cm] = rgb2ind(im,256);
                
                if start_animation_gif
                    imwrite(imind,cm,filename,'gif', 'Loopcount',inf,...
                        'DelayTime',S.animation_time_discretization) ;
                    start_animation_gif = false ;
                else
                    imwrite(imind,cm,filename,'gif','WriteMode','append',...
                        'DelayTime',S.animation_time_discretization) ;
                end
            else
                pause(S.animation_time_discretization)
            end
        end
        
        if hold_check
            hold off
        end
    end
    
    function filename = animation_gif_setup(S)       
        filename = S.save_gif_filename ;

        dir_content = dir(pwd) ;
        filenames   = {dir_content.name} ;
        file_check  = any(cellfun(@(x) strcmp(filename,x),filenames)) ;
        filename_new = filename ;
        cur_int = 1 ;

        while file_check
            filename_new = [filename(1:end-4),'_',num2str(cur_int),filename(end-3:end)] ;
            file_check  = any(cellfun(@(x) strcmp(filename_new,x),filenames)) ;
            cur_int = cur_int + 1 ;
        end

        filename = filename_new ;
    end

    %% utility
    function A = get_agent(S,idx)
        if nargin < 2 || S.N_agents == 1
            A = S.agents{1} ;
        else
            A = S.agents{idx} ;
        end
    end
    
    function W = get_world(S,idx)
        W = S.worlds{idx} ;
    end
    
    function P = get_planner(S,idx)
        P = S.planners{idx} ;
    end
    
    function vdisp(S,s,l,use_header)
    % Display a string 's' if the verbosity is greater than or equal to
    % the level 'l'; by default, the level is set to 0 and the default
    % threshold for displaying a message is 1 (so messages do not
    % display by default)
        if nargin < 4
            use_header = true ;
            if nargin < 3
                l = 1 ;
            end
        end

        if S.verbose >= l
            if use_header
                disp(['S: ',s])
            else
                disp(s)
            end
        end
    end
    end
end
