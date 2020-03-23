classdef multiFRS_RTD_planner < planner
    properties
        % robot info
        max_speed
        n_agent_inputs
        n_agent_states
        agent_time % agent's time at the beginning of the current plan
        agent_state % agent's state at the beginning of the current plan
        
        % trajectory optimization info
        trajopt_problem % structure for online trajectory optimization problem
        lookahead_distance = 1 ; % used to make waypoints
        desired_speed
        latest_plan % structure for most recently planned trajectory
        current_waypoint % waypoint transormed into FRS coordinates
        braking_flag % is true when trajopt failed
        timeout_flag % is true when trajopt fails to solve within t_plan
        FRS_buffer % amount to buffer obstacles for planning with the FRS
        
        % structure to store FRSes that are used during planning
        FRS_directory
        FRS
        current_FRS_index = 1 ; % in case multiple FRSes are used for planning
        FRS_indicies_to_optimize_over;
        current_obstacles_in_FRS_coordinates;
        
        % plotting
        plot_obstacle_flag = false ;
        plot_FRS_flag = false ;
        plot_waypoints_flag = true;
        
    end
    methods
        %% constructor
        function P = multiFRS_RTD_planner(varargin)
            % set default values
            buffer = 1 ;
            
            % parse input args
            P = parse_args(P,'buffer',buffer,varargin{:}) ;
            
            % instantiate FRS and high level planner
            P.load_FRS_files()
            P.initialize_high_level_planner()
        end
        
        function initialize_high_level_planner(P)
            % P.initialize_high_level_planner()
            %
            % Instantiate the high level planner to be used for online
            % planning. This is, by default, a straight_line_HLP instance,
            % which just puts a waypoint along a straight line between the
            % agent and the global goal. Overwrite this method in a
            % subclass to use different high level planners.
            
            if isempty(P.HLP)
                P.HLP = straight_line_HLP() ;
            end
        end
        
        function load_FRS_files(P,reload_FRS_files,variable_names_to_load)
            % P.load_FRS_files(reload_FRS_files)
            %
            % Find all .mat files in P.FRS_filepath, figures out which ones
            % actually contain FRS info, and load them in the P.FRS structure.
            %
            % The first input, reload_FRS_files, does exactly what it
            % sounds like. The FRS files may be large, so it's not a good
            % idea to load them over and over, in case one needs to use
            % this method to do other things (like sorting the FRSes).
            %
            % The second input, variable_names_to_load, should be specified
            % as a cell array of strings that are the variable names to
            % load from the saved .mat file; i.e., to load variables a and
            % b, one should pass in {'a','b'} as the second argument to
            % P.load_FRS_files
            %
            % For any RTD planner instance, the user should probably modify
            % this function to sort and/or parse the loaded FRS files in the
            % subclass by writing the following method:
            %
            % function load_FRS_files(P,reload_FRS_files,variable_names_to_load)
            %     load_FRS_files@RTD_planner(P,reload_FRS_files,variable_names_to_load)
            %
            %     <add your modifications here>
            % end
            
            if nargin < 2
                reload_FRS_files = true ;
            end
            
            if reload_FRS_files || isempty(P.FRS)
                P.vdisp('Loading FRS files',1)
                
                t_load = tic ;
                
                % make sure directory has a slash at the end
                if ~strcmp(P.FRS_directory(end),'/')
                    P.FRS_directory = [P.FRS_directory, '/'] ;
                end
                
                % get all files in the FRS directory
                files = dir(P.FRS_directory) ;
                
                % set up structure to store FRSes
                N_files = length(files) ;
                FRS_data = cell(1,N_files) ;
                
                % iterate through files and load each .mat file
                for idx = 1:N_files
                    current_filename = files(idx).name ;
                    current_filepath = [P.FRS_directory,current_filename] ;
                    
                    if ~strcmp(current_filename(1),'.') && ...
                            strcmp(current_filename(end-3:end),'.mat')
                        if nargin > 2
                            FRS_data{idx} = load(current_filepath,variable_names_to_load{:}) ;
                        else
                            FRS_data{idx} = load(current_filepath) ;
                        end
                    end
                end
                
                % remove any empty cells from the FRS data
                FRS_data = FRS_data(~cellfun('isempty',FRS_data)) ;
                
                % fill in planner's FRS field
                P.FRS = FRS_data ;
                
                % report time spent loading
                t_load = toc(t_load) ;
                P.vdisp(['Time spent loading FRS files: ',num2str(t_load),' s'],1)
            end
        end
        
        %% setup
        function setup(P,agent_info,world_info)
            % P.setup(agent_info,world_info)
            %
            % This method gets run once before running any simulation. It
            % lets the planner extract necessary info from the agent and
            % world, such as max speed, global goal location, and
            % boundaries.
            
            % get info needed from agent
            P.n_agent_inputs = agent_info.n_inputs ;
            P.n_agent_states = agent_info.n_states ;
            
            % trajectory optimization initialization
            P.braking_flag = true ;
            
            % reset latest plan trajectory info
            P.latest_plan.k_opt = nan(2,1) ;
            P.latest_plan.agent_state = [] ;
            P.latest_plan.time = [];
            P.latest_plan.input = [];
            P.latest_plan.trajectory = [];
            
            % get world bounds
            B = world_info.bounds ;
            P.bounds = B + P.buffer.*[1 -1 1 -1] ;
            
            % set up high level planner
            P.HLP.goal = world_info.goal ;
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            
            try
                P.HLP.bounds = P.bounds ;
            catch
                P.vdisp('High level planner has no bounds field.',9)
            end
            
            % data storage setup
            P.initialize_info_structure()
            
            % get current FRS index
            P.determine_current_FRS(agent_info)
            
            % plotting setup
            P.plot_data.obstacles = [] ;
            P.plot_data.waypoint = [] ;
            
            P.initialize_trajopt_problem();

        end
        
        %% setup: initialize trajopt problem
        function initialize_trajopt_problem(P)
            % P.initialize_trajopt_problem()
            %
            % Initialize trajopt problem structure for the RTD planner; this just
            % sets up fmincon options by default, so overwrite it to use a
            % different online solver.
            
            options =  optimoptions('fmincon',...
                'MaxFunctionEvaluations',1e5,...
                'MaxIterations',1e5,...
                'OptimalityTolerance',1e-3',...
                'CheckGradients',false,...
                'FiniteDifferenceType','central',...
                'Diagnostics','off',...
                'SpecifyConstraintGradient',true,...
                'SpecifyObjectiveGradient',true);
            
            P.trajopt_problem.cost_function = [] ;
            P.trajopt_problem.nonlcon_function = [] ;
            P.trajopt_problem.k_bounds = [] ;
            P.trajopt_problem.options = options ;
        end
        
        %% setup: initialize info structure
        function initialize_info_structure(P)
            % set up structure for info
            I = struct('agent_time',[],'agent_state',[],...
                'k_opt_found',[],...
                'agent_state_used_for_plan',[],...
                'current_FRS_index',[],...
                'FRS_index_used_for_plan',[],...
                'current_waypoint',[],...
                'current_obstacles',[],...
                'braking_flag',[],'planning_time',[]) ;
            P.info = I ;
        end
        
        %% online planning: replan
        function [T_out,U_out,Z_out] = replan(P,agent_info,world_info)
            % [T_out,U_out,Z_out] = P.replan(agent_info,world_info)
            %
            % The replan method takes the latest agent and obstacle information
            % and returns a sequence of inputs (U_out) with associated time (T_out)
            % and anticipated trajectory (Z_out) for the agent to execute over the
            % duration P.t_move. The planner self-enforces a time limit of
            % P.t_plan; if P.t_move = P.t_plan, then the planner must
            % create plans in real time.
            %
            % Probably, the user should not overwrite this method. Any of the
            % methods called by this one can be modified/overwritten by a subclass
            % of this generic RTD planner. In particular, one should overwrite
            % or modify the following methods:
            %     P.trajopt
            %     P.make_nominal_trajectory
            %     P.increment_nominal_trajectory
            %     P.make_stopped_control_input
            
             % get agent state
            P.agent_state = agent_info.state(:,end) ;
            
            % get agent time
            P.agent_time = agent_info.time(:,end);
            
            % figure out current FRS from agent's current state; this function sets
            % the current_FRS_index property (should be custom-written for any
            % RTD planner)
            P.determine_current_FRS(agent_info) ;
            
            P.create_waypoint(agent_info,world_info);
            
            % reset the timeout parameter
            P.timeout_flag = false ;
            
            k_opt = cell([1,length(P.FRS_indicies_to_optimize_over)]);
            final_cost = Inf([1,length(P.FRS_indicies_to_optimize_over)]);
            exitflag = -ones([1,length(P.FRS_indicies_to_optimize_over)]);
            traj_opt_time = NaN([1,length(P.FRS_indicies_to_optimize_over)]);
            
            P.current_obstacles = cell(1,length(P.FRS_indicies_to_optimize_over));
            P.current_obstacles_in_FRS_coordinates = cell(1,length(P.FRS_indicies_to_optimize_over));
            
        if ~isempty(k_opt)
        
        for i = 1:length(P.FRS_indicies_to_optimize_over)
            
            % create problem structure
            P.prep_for_trajopt(agent_info,world_info,i)

            % call online optimizer if we haven't reached the timeout
            [k_opt{i},final_cost(i),exitflag(i),traj_opt_time(i)] = P.trajopt() ;
            
        end
        
        
        [selected_frs_idx] = P.select_k_opt(k_opt,final_cost,exitflag);
        
        P.current_obstacles = P.current_obstacles{selected_frs_idx};
        P.current_obstacles_in_FRS_coordinates = P.current_obstacles_in_FRS_coordinates{selected_frs_idx};
        
        exitflag_select = exitflag(selected_frs_idx);
        if exitflag_select <=0
            
            traj_opt_time_select = NaN;
            k_opt_select = NaN(size(k_opt{selected_frs_idx}));
            P.current_FRS_index = selected_frs_idx;

        else
            
            k_opt_select = k_opt{selected_frs_idx};
            traj_opt_time_select = traj_opt_time(selected_frs_idx);
            P.current_FRS_index = P.FRS_indicies_to_optimize_over(selected_frs_idx);
            
        end
     else
        
        k_opt_select = NaN;
        exitflag_select = -1;
        traj_opt_time_select = NaN;
        P.current_FRS_index = NaN;
        P.current_obstacles = [];
        P.current_obstacles_in_FRS_coordinates = [];

    end
    

            % parse output
            [T_out,U_out,Z_out] = P.parse_trajopt_output(agent_info,k_opt_select,exitflag_select) ;
            
            % save info
            P.save_online_planning_info(traj_opt_time_select)
        end
        
        function selected_frs_idx = select_k_opt(P,k_opt,final_cost,exitflag)
            
            [~,selected_frs_idx] = min(final_cost);
            
        end
        
        %% online planning: setup trajopt problem
        function prep_for_trajopt(P,agent_info,world_info,frs_idx)
            % P.prep_for_trajopt(A,O,t_replan_start)
            %
            % Fill in the P.trajopt_problem structure with the following steps:
            %   1. determine the current FRS
            %   2. process the current obstacles (i.e. buffer and discretize)
            %   3. create the cost and constraint functions for the current plan
            %
            % Each of the methods called by this method should be custom-written in
            % a subclass of RTD_planner

            % process world (obstacle) info (should be custom written)
            P.process_world_info(world_info,frs_idx) ;
            
            % create constraints
            P.create_constraints(frs_idx) ;
            
            % create cost (note that, inside P.create_cost_function, the user
            % should typically call P.create_waypoint)
            P.create_cost_function(agent_info,world_info,frs_idx) ;
            
            % create initial guess
            P.create_initial_guess(frs_idx) ;
        end
        
        %% online planning: create cost
        function create_cost_function(P,agent_info,world_info,idx)
            P.vdisp(['Cost function undefined! Please write a ',...
                'create_cost_function method for your RTD planner.'])

            P.trajopt_problem.cost_function = [] ;
        end
        
        %% online planning: create waypoint
        function create_waypoint(P,agent_info,world_info)
            % P.create_waypoint(agent_info,start_tic)
            %
            % This gets a waypoint from the high level planner (P.HLP) and
            % stores it in the P.current_waypoint field. It can be
            % overwritten to do more fancy things, such as putting the
            % waypoint into the agent's body-fixed frame.
            
            wp = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
            
            P.current_waypoint = wp ;
        end
        
        %% online planning: create initial guess
        function create_initial_guess(P,frs_idx)
            % P.create_initial_guess(start_tic)
            %
            % By default, set the initial guess for trajopt to the previous k_opt,
            % or else zeros. This is a useful one to overwrite in a subclass,
            % probably.
            
            k_prev = P.latest_plan.k_opt ;
            
            if isempty(k_prev) || any(isnan(k_prev))
                k = P.FRS{P.current_FRS_index}.k ;
                P.trajopt_problem.initial_guess = zeros(size(k)) ;
            else
                P.trajopt_problem.initial_guess = k_prev ;
            end
        end
        
        %% online planning: create nonlcons and linear cons
        function create_constraints(P,frs_idx)
            % P.create_constraints(start_tic)
            %
            % This should take whatever is in P.current_obstacles and turn
            % it into constraints for online planning. It should also
            % create bounds on the trajectory parameter space, and any
            % other constraints that P.trajopt needs.
            
            P.vdisp(['Constraints undefined! Please write a ',...
                'create_constraints method for your RTD planner.'],1)
            
            P.trajopt_problem.nonlcon_function = [] ;
            
            k = P.FRS{P.current_FRS_index}.k ;
            
            P.trajopt_problem.Aineq = [] ;
            P.trajopt_problem.bineq = [] ;
            P.trajopt_problem.k_bounds = [-ones(size(k)),ones(size(k))] ;
        end
        
        %% online planning: determine current FRS
        function determine_current_FRS(P,agent_info)
            % P.determine_current_FRS(agent_info)
            %
            % Since multiple FRSes can be used for planning, but we should
            % probably only pick one at each planning iteration, this
            % method lets the RTD planner pick one based on the agent's
            % state. For example, we often choose the FRS that has the
            % highest available commanded speed corresponding to the
            % agent's initial condition.
            
            P.vdisp(['Please define the RTD_planner''s determine_current_FRS ',...
                'method! You are using the default method, which sets ',...
                'the current FRS index to 1.'],10)
            P.current_FRS_index = 1 ;
            P.FRS_indicies_to_optimize_over = 1;
            %initial trajectory optimization problem for number of FRS's
            %you want to check
        end
        
        
        %% online planning: process obstacles
        function process_world_info(P,world_info,frs_idx)
            % P.process_world_info(world_info)
            %
            % By default, this method just puts the world_info.obstacles
            % structure directly into the planner's current_obstacles
            % field; it should be modified to do more fancy things if they
            % are needed
            
            % update planner object
            P.current_obstacles = world_info.obstacles ;
        end
        
        %% online planning: trajopt
        function [k_opt,feval,exitflag,trajopt_time] = trajopt(P)
            % [k_opt,exitflag] = P.trajopt()
            %
            % Run the online optimizer. By default, this calls fmincon with the cost
            % function defined by P.create_cost_function, the nonlinear constraints
            % created by P.create_constraints, the linear constraints
            % created by P.create_trajopt_bounds_and_ineq_cons, and the initial guess
            % created by P.create_initial_guess

                   try
                    start_tic = tic;
                        
                    if ~isempty(P.trajopt_problem.nonlcon_function)
                        [k_opt,feval,exitflag] = fmincon(@(x) P.trajopt_problem.cost_function(x,start_tic),...
                            P.trajopt_problem.initial_guess,...
                            P.trajopt_problem.Aineq,...
                            P.trajopt_problem.bineq,...
                            [],[],... % linear equality constraints
                            P.trajopt_problem.k_bounds(:,1),...
                            P.trajopt_problem.k_bounds(:,2),...
                            @(x) P.trajopt_problem.nonlcon_function(x,start_tic),...
                            P.trajopt_problem.options) ;
                    else
                        P.vdisp('constraint function is empty',4)
                        [k_opt,feval,exitflag] = fmincon(@(x) P.trajopt_problem.cost_function(x,start_tic),...
                            P.trajopt_problem.initial_guess,...
                            P.trajopt_problem.Aineq,...
                            P.trajopt_problem.bineq,...
                            [],[],... % linear equality constraints
                            P.trajopt_problem.k_bounds(:,1),...
                            P.trajopt_problem.k_bounds(:,2),...
                            [],...
                            P.trajopt_problem.options) ;
                    end
                    
                 catch
                     P.vdisp('optimization error',1)
                     k_opt = nan(size(P.trajopt_problem.initial_guess)) ;
                     exitflag = -1 ;
                  end 
                  trajopt_time = toc(start_tic);
                    
                    if exitflag<0
                        feval = Inf;
                    end
       
        end
        
        %% online planning: parse trajopt output
        function [T_out,U_out,Z_out] = parse_trajopt_output(P,agent_info,k_opt,exitflag)
            % [T_out,U_out,Z_out] = P.parse_trajopt_output(agent_info,k_opt,exitflag)
            %
            % Given an optimal parameter and exitflag from the trajopt method,
            % produce a time, input, and desired trajectory to be passed to the
            % robot.
            
            if exitflag > 0
                P.vdisp('Found a feasible solution!',2)
                P.braking_flag = false ;
                
                P.latest_plan.k_opt = k_opt ;
                
                % create the time and input; by default, this is just k_opt held
                % for the current FRS time horizon
                [T_out,U_out,Z_out] = P.create_desired_trajectory(agent_info,k_opt) ;
                
                % transform the output to the world frame
                P.vdisp('Transforming new plan to world frame',7)
                idx = [agent_info.position_indices(:) ; agent_info.heading_index] ;
                Z_out(idx,:) = local_to_world(agent_info.state(idx,end),Z_out(idx,:)) ;
                            

            else
                P.vdisp('No feasible solution found!',2)
                P.braking_flag = true ;
                
                % note that the k_opt, agent_state, and FRS_index in the
                % P.latest_plan structure are left alone here
                
                % if there is a previously-found trajectory, subtract P.t_move from
                % the time vector and return the remainder of the previously-found
                % trajectory; if there is not more than t_move of the previous
                % trajectory left, then the robot should be stopped, so return a
                % control input that keeps it stopped
                if isempty(P.latest_plan.time) || P.latest_plan.time(end) < P.t_move
                    [T_out,U_out,Z_out] = P.create_stopped_control_input(agent_info) ;
                else
                    [T_out,U_out,Z_out] = P.increment_previous_trajectory(agent_info) ;
                end
                
                P.latest_plan.k_opt = nan(size(P.trajopt_problem.initial_guess)) ;
            end
            P.latest_plan.time = T_out ;
            P.latest_plan.input = U_out ;
            P.latest_plan.trajectory = Z_out ;
            P.latest_plan.agent_state = P.agent_state ;
            P.latest_plan.current_FRS_index = P.current_FRS_index ;

        end
        
        %% online planning: create output given successful replan
        function [T_out,U_out,Z_out] = create_desired_trajectory(P,agent_info,k_opt)
            % P.create_desired_trajectory(agent_info,k_opt)
            %
            % This method should be used to generate the desired trajectory
            % corresponding to k_opt. It just returns a time vector and the
            % optimal parameter k_opt by default, so it should be
            % overwritten in any subclass.
            
            P.vdisp('Please define the P.create_desired_trajectory method!')
            
            T_out = [0, P.FRS{P.current_FRS_index}.T] ;
            U_out = [k_opt, k_opt] ;
            Z_out = [] ;
        end
        
        %% online planning: create output given unsuccessful replan
        function [T_out,U_out,Z_out] = increment_previous_trajectory(P,agent_info)
            % P.[T_out,U_out,Z_out] = increment_previous_trajectory(agent_info)
            %
            % If trajectory optimization fails during one iteration, then
            % this method increments the previously-found trajectory by the
            % duration P.t_move so that it can be passed to the agent to
            % continue executing the previous trajectory. This can also
            % generate a braking/stopped trajectory if there is no
            % previously-found trajectory available.
            
            % get previous trajectory
            T_old = P.latest_plan.time ;
            U_old = P.latest_plan.input ;
            Z_old = P.latest_plan.trajectory ;
            
            % find portion of previous trajectory corresponding to time greater
            % than or equal to t_move
            T_log = T_old >= P.t_move ;
            
            % increment the time and input
            T_out = T_old(T_log) - P.t_move ;
            U_out = U_old(:,T_log) ;
            
            % increment the desired trajectory if it isn't empty
            if ~isempty(Z_old)
                Z_out = Z_old(:,T_log) ;
            else
                Z_out = [] ;
            end
            
            % if the first value of T_out is not zero, add a column of zeros in
            % front and interpolate the input and desired traj to match
            if T_out(1) > 0
                T_out = [0,T_out] ;
                U_zro = match_trajectories(P.t_move,T_old,U_old,'pchip') ;
                U_out = [U_zro, U_out] ;
                
                if ~isempty(Z_old)
                    Z_zro = match_trajectories(P.t_move,T_old,Z_old,'pchip') ;
                    Z_out = [Z_zro, Z_out] ;
                end
            end
            
            % if the new output is only one time instance long, append a control
            % input to keep the agent stopped
            if length(T_out) == 1
                [T_stp,U_stp,Z_stp] = P.create_stopped_control_input(agent_info) ;
                
                if T_out(end) < P.t_move
                    T_out = [T_out, T_stp + P.t_move] ;
                    U_out = [U_out, U_stp] ;
                    
                    if ~isempty(Z_out)
                        Z_out = [Z_out, repmat(Z_out(:,end),1,2)] ;
                    end
                else
                    T_out = T_stp ;
                    U_out = U_stp ;
                    
                    if ~isempty(Z_old)
                        Z_out = Z_stp ;
                    end
                end
            end
        end
        
        %% online planning: create control input to keep robot stopped
        function [T_out,U_out,Z_out] = create_stopped_control_input(P,agent_info)
            % [T_out,U_out,Z_out] = P.create_stopped_control_input(agent_info)
            %
            % This creates a desired trajectory that should keep the agent
            % stopped; by default, it returns a duration of P.t_move with
            % zeros for the control input along with the agent's last
            % reported position, with all other states set to zero. This
            % should definitely be overwritten in a subclass.
            
            T_out = [0, P.t_move] ;
            U_out = zeros(P.n_agent_inputs,2) ;
            pos_idx = agent_info.position_indices ;
            Z_out = zeros(P.n_agent_states,1) ;
            Z_out(pos_idx) = agent_info.state(pos_idx,end) ;
            Z_out = repmat(Z_out,1,2) ;
        end
        
        %% online planning: save planning info
        function save_online_planning_info(P,timing)
            % P.save_online_planning_info()
            %
            % This updates the P.info object with everything corresponding
            % to the latest plan, such as the agent pose, time, and desired
            % trajectory. This should usually be called by a subclass
            % method and then modified with additional information, instead
            % of overwriting this method.
            
            I = P.info ;
            
            I.agent_time = [I.agent_time,P.agent_time];
            
            I.agent_state = [I.agent_state, P.agent_state] ;
            
            if isempty(P.latest_plan.agent_state)
                I.agent_state_used_for_plan = [I.agent_state_used_for_plan, NaN(size(P.agent_state))] ;
            else
                I.agent_state_used_for_plan = [I.agent_state_used_for_plan, P.latest_plan.agent_state] ;
            end
            
            I.k_opt_found = [I.k_opt_found, P.latest_plan.k_opt] ;
            
            I.current_FRS_index = [I.current_FRS_index, P.current_FRS_index] ;
            
            if isempty(P.latest_plan.current_FRS_index)
                I.FRS_index_used_for_plan = [I.FRS_index_used_for_plan, NaN] ;
            else
                I.FRS_index_used_for_plan = [I.FRS_index_used_for_plan, P.latest_plan.current_FRS_index] ;
            end
            
            I.current_waypoint = [I.current_waypoint, P.current_waypoint] ;
            
            I.current_obstacles = [I.current_obstacles, {P.current_obstacles}] ;
            
            I.braking_flag = [I.braking_flag, P.braking_flag] ;
            
            I.planning_time = [I.planning_time,timing];
            
            P.info = I ;
        end
      
    end
end