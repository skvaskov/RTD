classdef segway_NMPC_planner < segway_generic_planner
% Class: segway_NMPC_planner < planner
%
% Implements nonlinear model-predictive control (NMPC) as a trajectory
% planner with GPOPS-II
%
% Authors: Shreyas Kousik, Sean Vaskov, and Ram Vasudevan
% Created: some time in 2018
% Updated: 21 Mar 2020
    
    %% properties
    properties
        % GPOPS-specific properties
        gpops_problem % problem structure to use
        parallel_cluster % so we can run jobs with a timeout
        spin_flag = true ; % spin to face the first waypoint in the first iteration
        use_coarse_initial_guess_flag
        t_min = 0.5 ; % lower bound on timing, can be used to encourage safety
        t_max = 2 ;% upper bound on timing, encourages robot to go faster
    end
    
    %% methods
    methods
        %% constructor
        function P = segway_NMPC_planner(varargin)
            % set default properties
            name = 'Segway NMPC Planner' ;
            t_move = 0.5 ;
            t_plan = 10 ;
            buffer = 0.5 ;
            HLP = straight_line_HLP() ; % default high level planner
            
            % create planner
            P = parse_args(P,'t_move',t_move,'t_plan',t_plan,...
                'buffer',buffer,'name',name,'HLP',HLP,varargin{:}) ;
        end
        
        %% setup
        function setup(P,agent_info,world_info)
        % 1. set up info from agent and world
            P.agent_max_accel = agent_info.max_accel ;
            P.agent_max_yaw_rate = agent_info.max_yaw_rate ;
            b = P.buffer + agent_info.footprint ;
            P.bounds = world_info.bounds + [b -b b -b] ;
            
        % 2. set up high-level planner
            P.HLP.setup(agent_info,world_info) ;
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            
        % 3. create GPOPS problem object
            P.gpops_problem = P.make_GPOPS_problem_object(agent_info,world_info) ;
            
        % 4. set up to spin towards waypoint on first iteratino
            P.spin_flag = true ;
            
        % 5. initialize current plan
            P.current_plan.T = [] ;
            P.current_plan.U = [] ;
            P.current_plan.Z = [] ;
            
        % 6. call IPOPT once to prevent errors (this is a bug in GPOPS)
            try
                ipopt
            catch
            end
            
        % 7. set up info structure
            P.info = struct('agent_time',[],'agent_state',[],...
                'waypoint',[],...
                'obstacles',[],...
                'T',[],'U',[],'Z',[]) ;
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
        % 1. get agent info
            z_cur = agent_info.state(:,end) ;
            P.agent_average_speed = get_segway_average_speed(agent_info.time,...
                agent_info.state(agent_info.speed_index,:),...
                P.agent_average_speed_time_horizon) ;
            
        % 2. set up obstacles
            [O_buf,A_O,b_O,N_obs,N_halfplanes] = P.process_obstacles(world_info) ;
            
        % 3. get waypoint
            lkhd = (P.agent_average_speed + P.lookahead_distance) / 2 ;
            world_info.obstacles = O_buf ;
            z_goal = P.HLP.get_waypoint(agent_info,world_info,lkhd) ;
            P.current_waypoint = z_goal ;
            
        % 4. set up GPOPS problem
            % initial guess
            [T_guess,U_guess,Z_guess] = P.make_GPOPS_initial_guess(agent_info,...
                z_goal,P.use_coarse_initial_guess_flag) ;
            P.gpops_problem.guess.phase.time = T_guess(:) ;
            P.gpops_problem.guess.phase.control = U_guess' ;
            P.gpops_problem.guess.phase.state = Z_guess' ;
            
            P.gpops_problem.bounds.phase.initialstate.lower = z_cur' ;
            P.gpops_problem.bounds.phase.initialstate.upper = z_cur' ;
            
            P.gpops_problem.auxdata.obs.A = A_O ;
            P.gpops_problem.auxdata.obs.b = b_O ;
            P.gpops_problem.auxdata.obs.N_obs = N_obs ;
            P.gpops_problem.auxdata.obs.N_halfplanes = N_halfplanes ;
            
            
            % goal
            P.gpops_problem.auxdata.goal_position = z_goal ;
            P.gpops_problem.auxdata.goal_heading = atan2(z_goal(2) - z_cur(2), z_goal(1) - z_cur(1)) ;
            
        % 5. run GPOPS
            P.gpops_problem.auxdata.start_tic = tic ;
            
%             try
                P.vdisp('Running GPOPS.',5)
                gp = P.gpops_problem ;
                output = gpops2(gp) ;
%             catch
%                 P.vdisp('GPOPS errored!',5)
%                 output = [] ;
%             end
        
        % 6. process trajectory plan
            [T,U,Z] = P.process_traj_opt_result(output,agent_info) ;
            
        % 7. update P.info
            P.current_plan.T = T ;
            P.current_plan.U = U ;
            P.current_plan.Z = Z ;
        end
        
        %% replan: process obstacles
        function [O_buf,A_O,b_O,N_obs,N_halfplanes] = process_obstacles(P,world_info)
            % create buffered obstacles
            O = world_info.obstacles ;
            O_buf = [] ;
            A_O = [] ;
            b_O = [] ;
            
            % make sure O does not have a column of nans to start
            if isnan(O(1,1))
                O = O(:,2:end) ;
            end
            
            % create buffered obstacles and halfplane representations
            N_O = size(O,2) ;
            N_obs = ceil(N_O/6) ;
            for idx = 1:6:N_O
                % get obstacle (we know it's a written as 5 points in CCW
                % order, so we can cheat a bit here)
                o = O(:,idx:idx+4) ;
                
                % create buffered obstacles
                o_buf = buffer_box_obstacles(o,P.buffer,13) ;
                O_buf = [O_buf, nan(2,1), o_buf] ;
                
                % create halfplane representation for collision checking
                [A_idx,b_idx] = vert2lcon(o_buf') ;
                A_O = [A_O ; A_idx] ;
                b_O = [b_O ; b_idx] ;
                
                % get the number of halfplanes (thank goodness this ends up
                % being exactly the same for every obstacle, saving us a
                % lot of work)
                N_halfplanes = length(b_idx) ;
            end
            
            P.current_obstacles = O_buf ;
        end
        
        %% replan: make initial guess
        function [T,U,Z] = make_GPOPS_initial_guess(P,agent_info,z_goal,make_coarse_guess)
            P.vdisp('Making initial guess for GPOPS',5)
            
            z_cur = agent_info.state(:,end) ;
            
            if make_coarse_guess
                P.vdisp('Using coarse initial guess!',6)
                
                h_goal = atan2(z_goal(2) - z_cur(2), z_goal(1) - z_cur(1)) ;
                z_goal = [z_goal(1:2) ; h_goal ; zeros(2,1)] ;
                
                T = [0, P.lookahead_distance./P.agent_max_speed] ;
                U = zeros(2,2) ;
                Z = [z_cur, z_goal] ;
            else
                P.vdisp('Using previous plan as initial guess!',6)
                
                [T,U,Z,stop_flag] = P.increment_plan(agent_info) ;
                
                % if the output just says to stay stopped, then use a
                % coarse initial guess
                if stop_flag
                    [T,U,Z] = make_GPOPS_initial_guess(P,agent_info,z_goal,true) ;
                end
            end
        end
        
        %% replan: process trajopt output
        function [T,U,Z] = process_traj_opt_result(P,output,agent_info)
            if ~isempty(output)
                P.vdisp('GPOPS found a solution',4)
                solution = output.result.solution ;
                
                if output.result.solution.phase.time(end) >= P.t_min
                    T = solution.phase.time' ;
                    U = solution.phase.control' ;
                    Z = solution.phase.state' ;
                else
                    [T,U,Z] = P.increment_plan(agent_info) ;
                end
            else
                P.vdisp('GPOPS did not find a solution',4)
                [T,U,Z] = P.make_plan_for_traj_opt_failure(agent_info) ;
            end
        end
        
        %% plotting
        function plot(P,~)
            hold_check = hold_switch() ;
            
            % plot trajectory plan
            Z = P.current_plan.Z ;
            if ~isempty(Z)
                if check_if_plot_is_available(P,'trajectory')
                    P.plot_data.trajectory.XData = Z(1,:) ;
                    P.plot_data.trajectory.XYData = Z(2,:) ;
                else
                    d = plot_path(Z(1:2,:),'b--') ;
                    P.plot_data.trajectory = d ;
                end
            end
            
            % plot obstacles
            O = P.current_obstacles ;
            if ~isempty(O)
                if check_if_plot_is_available(P,'obstacles')
                    P.plot_data.obstacles.XData = O(1,:) ;
                    P.plot_data.obstacles.YData = O(2,:) ;
                else
                    d = plot_path(O,'-','color',[1 0.5 0.5]) ;
                    P.plot_data.obstacles = d ;
                end
            end
            
            % plot waypoint
            waypoint = P.current_waypoint ;
            if P.plot_waypoints_flag && ~isempty(waypoint)
                if check_if_plot_is_available(P,'waypoint')
                    P.plot_data.waypoint.XData = waypoint(1) ;
                    P.plot_data.waypoint.YData = waypoint(2) ;
                else
                    d = plot_path(waypoint,'kp') ;
                    P.plot_data.waypoint = d ;
                end
            end
            
            if P.plot_HLP_flag
                plot(P.HLP) ;
            end
            
            hold_switch(hold_check) ;
        end
        
        %% make GPOPS problem object
        function out = make_GPOPS_problem_object(P,agent_info,world_info)
            bounds.phase.initialtime.lower = 0 ;
            bounds.phase.initialtime.upper = 0 ;
            bounds.phase.finaltime.lower   = P.t_min ;
            bounds.phase.finaltime.upper   = P.t_max ;
            
            % problem bounds
            B = P.bounds ;
            x_min = B(1) ;
            x_max = B(2) ;
            y_min = B(3) ;
            y_max = B(4) ;
            h_min = -Inf ;
            h_max = +Inf ;
            w_min = -agent_info.max_yaw_rate ;
            w_max = +agent_info.max_yaw_rate ;
            v_min = 0 ;
            v_max = agent_info.max_speed ;
            
            bounds.phase.state.lower        = [ x_min, y_min, h_min, w_min, v_min ];
            bounds.phase.state.upper        = [ x_max, y_max, h_max, w_max, v_max ];
            bounds.phase.finalstate.lower   = -Inf * ones(1, 5);
            bounds.phase.finalstate.upper   = Inf * ones(1, 5);
            bounds.phase.control.lower = [w_min, v_min];
            bounds.phase.control.upper = [w_max, v_max];
            bounds.phase.path.lower = 0 ;
            bounds.phase.path.upper = Inf ;
            
            % default initial guess
            t_guess    = [0; 1 ];
            w_guess    = [0; 0 ];
            v_guess    = [0; 0 ];
            guess.phase.time = t_guess ;
            guess.phase.control = [w_guess,v_guess];
            
            % mesh settings
            mesh.method = 'hp-LiuRao-Legendre';
            mesh.tolerance = 1e-6 ;
            mesh.maxiterations = 1000 ;
            mesh.colpointsmin = 4 ;
            mesh.colpointsmax = 10 ;
            mesh.phase.colpoints = 4*ones(1,10);
            mesh.phase.fraction = 0.1*ones(1,10);
            
            % obstacle gpops_problem
            auxdata.location_weight = 1 ;
            auxdata.heading_weight = 0.1 ;
            auxdata.speed_weight = 1 ;
            auxdata.yaw_rate_weight = 0.1 ;
            auxdata.start_tic = tic ;
            auxdata.timeout = P.t_plan ;
            
            % finalize setting up GPOPS problem object
            out.bounds = bounds ;
            out.guess = guess ;
            out.mesh = mesh ;
            out.name = 'Segway GPOPS output' ;
            out.functions.continuous = @gpops_segway_dynamics ;
            out.functions.endpoint = @gpops_segway_endpoint ;
            out.auxdata = auxdata ;
            out.nlp.solver = 'ipopt' ;
            out.displaylevel = 0 ;
            out.derivatives.derivativelevel = 'second' ;
            out.derivatives.supplier = 'sparseCD' ;
        end
    end
end


function out = gpops_segway_dynamics(input)
% dynamics
    x = input.phase.state(:, 1);
    y = input.phase.state(:, 2);
    h = input.phase.state(:, 3);
    w = input.phase.state(:, 4);
    v = input.phase.state(:, 5);

    k_1 = input.phase.control(:, 1);
    k_2 = input.phase.control(:, 2);

    % yaw rate input
    wdes = k_1 ;
    Kg = 2.95 ;
    g = Kg*(wdes - w) ;

    % acceleration input
    vdes = k_2 ;
    vdelta = vdes - v ;
    Ka = 3 ;
    a = Ka*vdelta ;

    x_dot     = v .* cos(h) ;
    y_dot     = v .* sin(h) ;
    theta_dot = w ;
    omega_dot = g ;
    v_dot = a ;

    out.dynamics  = [x_dot, y_dot, theta_dot, omega_dot, v_dot];

% obstacle check
    % get the obstacle halfplane representation
    A = input.auxdata.obs.A ;
    b = input.auxdata.obs.b ;
    N_obs = input.auxdata.obs.N_obs ;
    N_halfplanes = input.auxdata.obs.N_halfplanes ;
    
    % get the points along the trajectory
    N_pts = length(x) ;
    X = [x y]' ;
    
    % check all the points (note, the last line of this check would usually
    % be -min(.), but GPOPS needs the opposite, where constraints are
    % negative when they are satisfied, for some reason)
    X_chk = A*X - b ;
    X_chk = reshape(X_chk,N_halfplanes,[]) ;
    X_chk = reshape(max(X_chk,[],1),N_obs,[]) ;
    X_chk = min(X_chk,[],1) ;
    
    % assign output
    out.path = X_chk(:) ;
    
    if timeout_check(input)
        out.dynamics = zeros(N_pts,5) ;
        error('timed out!')
%         out.path = zeros(size(half_plane_check)) ;
%     else
%         
    end
%     timeout_check(input) ;
end

function out = gpops_segway_endpoint(input)
%         if timeout_check(input)
%             out.objective = 0 ;
%         else
        x = input.phase.finalstate(1);
        y = input.phase.finalstate(2);
        h = input.phase.finalstate(3);
        w = input.phase.finalstate(4);
        v = input.phase.finalstate(5);

        x_goal = input.auxdata.goal_position(1) ;
        y_goal = input.auxdata.goal_position(2) ;
        h_goal = input.auxdata.goal_heading ;
        
        location_weight = input.auxdata.location_weight ;
        heading_weight = input.auxdata.heading_weight ;
        yaw_rate_weight = input.auxdata.yaw_rate_weight ;
        speed_weight = input.auxdata.speed_weight ;

        out.objective = location_weight.*((x-x_goal).^2 + (y-y_goal).^2) + ...
            heading_weight.*((h - h_goal).^2) + ...
            yaw_rate_weight.*(w.^2) + ...
            speed_weight.*(v.^2)               ;
%         end
end

function out = timeout_check(input)
    out = toc(input.auxdata.start_tic) >= input.auxdata.timeout ;
end