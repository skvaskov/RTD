classdef segway_RTD_planner < planner
% Class: segway_RTD_planner < planner
%
% This class implements RTD for a segway in static environments. It does
% not inherit the generic RTD planner superclass, so that you can see how
% all the parts of a planner should be written in a simulator framework.
%
% Note, we have put all of the necessary "helper" functions in as static
% methods, to avoid conflicts with other RTD repositories.
%
% Author: Shreyas Kousik
% Created: 11 Mar 2020
% Updated: 14 Mar 2020

    %% properties
    properties
        % agent information
        agent_footprint = 0.38 ;
        agent_max_accel = 3 ;
        agent_max_yaw_rate = 2 ;
    
        % FRS handling
        FRS % this is a cell array of loaded info from FRS .mat files
        FRS_degree = 10 ; % set to 4, 6, 8, 10, or 12
        FRS_polynomial_structure
        
        % obstacle handling
        point_spacing
        current_obstacles_raw
        current_obstacles_in_FRS_coords
        bounds_as_obstacle
        
        % plan handling
        current_waypoint
        spin_in_place_flag = false ;
        lookahead_distance = 1.5 ; % meters
        
        % plotting
        plot_obstacles_flag = true ;
        plot_FRS_flag = false ;
        plot_waypoints_flag = false ;
    end
    
    %% class methods
    methods
    %% constructor
        function P = segway_RTD_planner(varargin)
            % P = segway_RTD_planner(varargin)
            %
            % This constructs the RTD planner.
            
            % set default values of some properties for the superclass that
            % are relevant to this particular planner; note that other
            % properties are defined in the planner "setup" method
            name = 'Segway RTD Planner' ;
            buffer = 1 ;
            HLP = straight_line_HLP() ; % default high level planner
            
            % parse the input arguments; these should be given in the
            % format 'property1', value1, 'property2', value2,...
            P = parse_args(P,'name',name,'buffer',buffer,'HLP',HLP,...
                           varargin{:}) ;
            
            % load FRS files
            FRS_data = cell(1,3) ;
            FRS_data{1} = P.get_FRS_from_v_0(0.0,P.FRS_degree) ;
            FRS_data{2} = P.get_FRS_from_v_0(0.5,P.FRS_degree) ;
            FRS_data{3} = P.get_FRS_from_v_0(1.0,P.FRS_degree) ;
            P.FRS = FRS_data ;
        end
        
    %% setup
        function setup(P,agent_info,world_info)
            % P.setup(agent_info, world_info)
            %
            % This is used to set stuff up before planning. For RTD, we use
            % it for the following:
            %   1. compute the obstacle discretization point spacing
            %   2. set up world boundaries as an obstacle
            %   3. give the high level planner the global goal info
            %   4. decompose the FRS polynomial into a usable form
            
            P.vdisp('Running setup',3)
            
        % 1. get properties needed from agent
            P.vdisp('Computing point spacing',4)
        
            bbar = agent_info.footprint ;
            b = P.buffer ;
            
            if b >= bbar
                P.buffer = bbar - 0.001 ;
                P.vdisp('Reducing buffer to be feasible!',2)
            end
            
            P.point_spacing = P.compute_point_spacings(bbar,P.buffer) ;
            
            P.agent_footprint = agent_info.footprint ;
            P.agent_max_accel = agent_info.max_accel ;
            P.agent_max_yaw_rate = agent_info.max_yaw_rate ;
            
        % 2. set up world boundaries as an obstacle
            P.vdisp('Setting up world bounds as an obstacle',4)
        
            P.bounds = world_info.bounds + P.buffer.*[1 -1 1 -1] ;
            
            % create world bounds as an obstacle; note this passes the
            % bounds in as a clockwise polyline, so everything outside of
            % the world bounds counts as inside the polyline if using
            % functions like inpolygon
            xlo = P.bounds(1) ; xhi = P.bounds(2) ;
            ylo = P.bounds(3) ; yhi = P.bounds(4) ;
            
            B = [xlo, xhi, xhi, xlo, xlo ;
                ylo, ylo, yhi, yhi, ylo] ;
            B = [B, nan(2,1), 1.01.*B(:,end:-1:1)] ;
            
            P.bounds_as_obstacle = B ;
            
        % 3. set up high level planner
            P.vdisp('Setting up high-level planner',4)
            
            P.HLP.setup(agent_info,world_info) ;
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            
        % 4. process the FRS polynomial
            P.vdisp('Processing FRS polynomial',4)
            
            P.FRS_polynomial_structure = cell(1,3) ;
            
            for idx = 1:3
                I = P.FRS{idx}.FRS_polynomial - 1 ;
                z = P.FRS{idx}.z ;
                k = P.FRS{idx}.k ;
                P.FRS_polynomial_structure{idx} = P.get_FRS_polynomial_structure(I,z,k) ;
            end
            
        % 5. initialize the current plan as empty
            P.vdisp('Initializing current plan',4)
            
            P.current_plan.T = [] ;
            P.current_plan.U = [] ;
            P.current_plan.Z = [] ;
            
        % 6. clear plot data
            P.plot_data.obstacles = [] ;
            P.plot_data.waypoint = [] ;
            P.plot_data.waypoints = [] ;
            P.plot_data.FRS = [] ;
            
        % 7. set up info structure to save replan dat
            I = struct('agent_time',[],'agent_state',[],...
                'k_opt_found',[],...
                'FRS_index',[],...
                'waypoint',[],...
                'waypoints',[],...
                'obstacles',[],...
                'obstacles_in_world_frame',[],...
                'obstacles_in_FRS_frame',[],...
                'spin_in_place_flag',[]) ;
            P.info = I ;
        end
        
    %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            % [T,U,Z] = P.replan(agent_info,world_info)
            %
            % This is the core of the RTD planner. In this method, we
            % generate a new trajectory plan, or continue the old plan, by
            % using the FRS to identify unsafe plans, and then using
            % an optimization program to find a safe plan.
            
            P.vdisp('Planning!',3)
            
            % start a timer to enforce the planning timeout P.t_plan
            start_tic = tic ;
            
            % 1. determine the current FRS based on the agent
            [FRS_cur,current_FRS_index,v_cur,w_cur] = get_current_FRS(P,agent_info) ;
            
            % 2. process obstacles
            [O,O_FRS,O_pts] = P.process_obstacles(agent_info,world_info,FRS_cur) ;
            
            % save obstacles
            P.current_obstacles_raw = O ; % input from the world
            P.current_obstacles = O_pts ; % buffered and discretized
            P.current_obstacles_in_FRS_coords = O_FRS ;
            
            % 3. create the cost function for trajectory optimization
            z_goal = P.get_waypoint(O,agent_info,world_info) ;
            cost = P.create_cost_function(FRS_cur,agent_info,z_goal,start_tic) ;
            
            % 4. create the constraints for fmincon
            nonlcon = P.create_constraint_function(O_FRS,current_FRS_index,start_tic) ;
            k_bounds = P.create_trajopt_bounds(w_cur,v_cur,FRS_cur) ;
            
            % 5. run trajectory optimization
            [k_opt,exitflag] = P.optimize_trajectory(cost,k_bounds,nonlcon) ;
            
            % 6. make the new plan, continue the old plan, or spin in place
            [T,U,Z] = P.process_traj_opt_result(k_opt,exitflag,agent_info,FRS_cur) ;
            
            % save the new plan
            P.current_plan.T = T ;
            P.current_plan.U = U ;
            P.current_plan.Z = Z ;
            
            % 7. update the info structure
            I = P.info ;
            
            I.agent_time = [I.agent_time, agent_info.time(end)] ;
            I.agent_state = [I.agent_state, agent_info.state(:,end)] ;
            I.k_opt_found = [I.k_opt_found, k_opt] ;
            I.FRS_index = [I.FRS_index, current_FRS_index] ;
            I.waypoint = [I.waypoint, z_goal] ;
            I.waypoints = [I.waypoints, {P.HLP.waypoints}] ;
            I.obstacles = [I.obstacles, {O}] ;
            I.obstacles_in_world_frame = [I.obstacles_in_world_frame, {O_pts}] ;
            I.obstacles_in_FRS_frame = [I.obstacles_in_FRS_frame, {O_FRS}] ;
            
            P.info = I ;
        end
        
        %% replan: get current FRS
        function [FRS_cur,current_FRS_index,v_cur,w_cur] = get_current_FRS(P,agent_info)
            P.vdisp('Determining current FRS',4)

            agent_state = agent_info.state(:,end) ; % (x,y,h,v)
            v_cur = agent_state(agent_info.speed_index) ;
            w_cur = agent_state(agent_info.yaw_rate_index) ;
            
            % pick fastest FRS for current speed
            if v_cur >= 1.0
                current_FRS_index = 3 ;
            elseif v_cur >= 0.5
                current_FRS_index = 2 ;
            else
                current_FRS_index = 1 ;
            end
            
            FRS_cur = P.FRS{current_FRS_index} ;
        end
        
        %% replan: process obstacles
        function [O,O_FRS,O_pts] = process_obstacles(P,agent_info,world_info,FRS_cur)
            P.vdisp('Processing obstacles',4)
        
            O = world_info.obstacles ;
            
            % add world bounds as obstacle
            O = [O, nan(2,1), P.bounds_as_obstacle] ;
            
            % buffer and discretize obstacles
            agent_state = agent_info.state(:,end) ;
            [O_FRS, ~, O_pts] = P.discretize_and_scale_obstacles(O,...
                    agent_state,P.buffer,P.point_spacing,FRS_cur) ;
        end
        
        %% replan: get waypoint
        function z_goal = get_waypoint(P,O,agent_info,world_info)
            P.vdisp('Getting waypoint',5)
            
            % buffer obstacles for high-level planner
            O_HLP = buffer_polygon_obstacles(O,P.agent_footprint) ;
            world_info.obstacles = O_HLP ;
            
            % make a waypoint (this is wrapped in a try/catch in case the
            % waypoint planner has bugs)
            try
                P.vdisp('Getting waypoint',7)
                z_goal = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
            catch
                P.vdisp('Waypoint creation errored! Using global goal instead',6)
                z_goal = P.HLP.goal ;
            end
            P.current_waypoint = z_goal ;
        end
        
        %% replan: make cost and constraints
        function cost = create_cost_function(P,FRS_cur,agent_info,z_goal,start_tic)
            P.vdisp('Creating cost function',4)
            
            % put waypoint in robot's body-fixed frame to use for planning
            z_goal_local = world_to_local(agent_info.state(1:3,end),z_goal(1:2)) ;
            
            % create cost function
            cost = @(k) segway_cost_for_fmincon(k,...
                            FRS_cur,z_goal_local,...
                            start_tic,P.t_plan) ;
        end
        
        function nonlcon = create_constraint_function(P,O_FRS,current_FRS_index,start_tic)
            P.vdisp('Creating constraints',4)
        
            % create nonlinear constraints from the obstacles
            if ~isempty(O_FRS)
                % remove NaNs
                O_log = isnan(O_FRS(1,:)) ;
                O_FRS = O_FRS(:,~O_log) ;
                
                % get FRS polynomial
                FRS_poly = P.FRS_polynomial_structure{current_FRS_index} ;

                % plug in to FRS polynomial
                cons_poly = evaluate_FRS_polynomial_on_obstacle_points(FRS_poly,O_FRS) ;

                % get the gradient of the constraint polynomials
                cons_poly_grad = get_constraint_polynomial_gradient(cons_poly) ;
                
                % create constraint function
                nonlcon = @(k) segway_nonlcon_for_fmincon(k,...
                                  cons_poly,cons_poly_grad,...
                                  start_tic,P.t_plan) ;
            else
                % if there are no obstacles then we don't need to consider
                % any constraints
                nonlcon = [] ;
            end
        end
        
        function k_bounds = create_trajopt_bounds(P,w_cur,v_cur,FRS_cur)
            % create bounds for yaw rate
            w_des_lo = max(w_cur - FRS_cur.delta_w, -FRS_cur.w_max) ;
            w_des_hi = min(w_cur + FRS_cur.delta_w, +FRS_cur.w_max) ;
            k_1_lo = w_des_lo ./ FRS_cur.w_max ;
            k_1_hi = w_des_hi ./ FRS_cur.w_max ;
            k_1_bounds = [k_1_lo, k_1_hi] ;

            % create bounds for speed
            v_max = FRS_cur.v_range(2) ;
            v_des_lo = max(v_cur - FRS_cur.delta_v, FRS_cur.v_range(1)) ;
            v_des_hi = min(v_cur + FRS_cur.delta_v, FRS_cur.v_range(2)) ;
            k_2_lo = (v_des_lo - v_max/2)*(2/v_max) ;
            k_2_hi = (v_des_hi - v_max/2)*(2/v_max) ;
            k_2_bounds = [k_2_lo, k_2_hi] ;

            % combine bounds
            k_bounds = [k_1_bounds ; k_2_bounds] ;
        end
        
        %% replan: run trajectory optimization
        function [k_opt,exitflag] = optimize_trajectory(P,cost,k_bounds,nonlcon)
            P.vdisp('Running trajectory optimization',4)
            
            % create initial guess
            initial_guess = [0;1] ; % no yaw rate, max velocity

            % create optimization options
            options =  optimoptions('fmincon',...
                            'MaxFunctionEvaluations',1e5,...
                            'MaxIterations',1e5,...
                            'OptimalityTolerance',1e-3',...
                            'CheckGradients',false,...
                            'FiniteDifferenceType','central',...
                            'Diagnostics','off',...
                            'SpecifyConstraintGradient',true,...
                            'SpecifyObjectiveGradient',true);
                        
            % call fmincon, with a try/catch since the cost and constraint
            % functions bail out by erroring if the timeout is reached
            try
                [k_opt,~,exitflag] = fmincon(cost,...
                                            initial_guess,...
                                            [],[],... % linear inequality constraints
                                            [],[],... % linear equality constraints
                                            k_bounds(:,1),... % lower bounds
                                            k_bounds(:,2),... % upper bounds
                                            nonlcon,...
                                            options) ;
            catch
                k_opt = nan(2,1) ;
                exitflag = -1 ;
            end
        end
        
        %% replan: process trajectory optimization
        function [T,U,Z] = process_traj_opt_result(P,k_opt,exitflag,agent_info,FRS_cur)
            P.vdisp('Creating plan from traj. opt. result',4)
            
            % if fmincon was successful, create a new plan
            if exitflag > 0
                P.vdisp('New plan successfully found!',5)
                w_des = full(msubs(FRS_cur.w_des,FRS_cur.k,k_opt)) ;
                v_des = full(msubs(FRS_cur.v_des,FRS_cur.k,k_opt)) ;

                % create the desired trajectory
                t_stop = v_des / P.agent_max_accel ;
                [T,U,Z] = make_segway_braking_trajectory(FRS_cur.t_plan,...
                            t_stop,w_des,v_des) ;
                        
                % move plan to world coordinates
                Z(1:3,:) = local_to_world(agent_info.state(:,end),Z(1:3,:)) ;
            else
            % if trajectory optimization was unsuccessful, try to continue
            % the previous plan
                P.vdisp('Continuing previous plan!',5)
                
                % first, check if there is enough of the past plan left to
                % keep executing
                T_old = P.current_plan.T ;
                U_old = P.current_plan.U ;
                Z_old = P.current_plan.Z ;
                
                if ~isempty(T_old)
                    % try shifting the current plan by P.t_move
                    T_log = T_old >= P.t_move ;
                else
                    T_log = false ;
                end
                   
                % keep what is left from the previous plan
                if any(T_log)
                    P.vdisp('Incrementing old plan!',6)
                    
                    % make sure P.t_move exists in the trajectory
                    T_temp = [T_old(T_old < P.t_move), P.t_move, T_old(T_old > P.t_move)] ;
                    [U,Z] = match_trajectories(T_temp,T_old,U_old,T_old,Z_old) ;
                    
                    % increment the time and input
                    T = T_old(T_log) - P.t_move ;
                    U = U(:,T_log) ;
                    Z = Z(:,T_log) ;
                else
                    P.vdisp('Making stopped plan!',6)
                    T = 0 ;
                    U = zeros(2,1) ;
                    Z = [agent_info.state(1:3,end) ; zeros(2,1)] ;
                end
                
                % make a spin-in-place maneuver of duration P.t_move
                P.vdisp('Adding spin maneuver to previous plan!',7)
                w_cur = agent_info.state(agent_info.yaw_rate_index,end) ;
                delta_w = FRS_cur.delta_w ;
                [T_spin,U_spin,Z_spin] = make_segway_spin_trajectory(P.t_move,...
                    w_cur,delta_w,P.agent_max_yaw_rate) ;
                
                T_spin = T_spin + T(end) ;
                
                % rotate and shift Z_spin to the end of the agent's current
                % trajectory
                Z_spin(1:3,:) = repmat(agent_info.state(1:3,end),1,size(Z_spin,2)) ;
                
                % add the spin maneuver to the end of the previous
                % trajectory
                T = [T, T_spin(2:end)] ;
                U = [U, U_spin(:,2:end)] ;
                Z = [Z, Z_spin(:,2:end)] ;
                
                % if the new plan is not long enough, add a spin-in-place
                % maneuver to the end of it
                if T(end) < P.t_move
                    T = [T, P.t_move, 2*P.t_move] ;
                    U = [U, zeros(2,1)] ;
                    Z = [Z, [Z(1:3,end) ; 0; 0], [Z(1:3,end) ; 0; 0] ] ;
                end
            end
        end
        
        %% plotting
        function plot(P,~)
            P.vdisp('Plotting!',8)
            P.plot_at_time() ;
        end
        
        function plot_at_time(P,t)            
            if nargin < 2
                if ~isempty(P.info.agent_time)
                    t = P.info.agent_time(end) ;
                else
                    t = 0 ;
                end
            end
            
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on ;
            end
            
            % figure out the info index closest to the current time
            I = P.info ;
            info_idx = find(t >= I.agent_time,1,'last') ;
            info_idx_check = ~isempty(info_idx) ;
            
            % plot current obstacles
            if P.plot_obstacles_flag && info_idx_check
                O = I.obstacles_in_world_frame{info_idx} ;

                if isempty(O)
                    O = nan(2,1) ;       
                end

                if check_if_plot_is_available(P,'obstacles')
                    P.plot_data.obstacles.XData = O(1,:) ;
                    P.plot_data.obstacles.YData = O(2,:) ;
                else
                    obs_data = plot(O(1,:),O(2,:),'r.') ;
                    P.plot_data.obstacles = obs_data ;
                end
            end
            
            % plot current waypoint
            if P.plot_waypoints_flag && info_idx_check
                wp = I.waypoint(:,info_idx) ;
                if isempty(wp)
                    wp = nan(2,1) ;
                end

                if check_if_plot_is_available(P,'waypoint')
                    P.plot_data.waypoint.XData = wp(1) ;
                    P.plot_data.waypoint.YData = wp(2) ;
                else
                    wp_data = plot(wp(1),wp(2),'b*') ;
                    P.plot_data.waypoint = wp_data ;
                end
            end
            
            % plot FRS
            if P.plot_FRS_flag && info_idx_check
                % iterate back through the info indices until the last
                % info index where k_opt was found successfully
                FRS_info_idx = info_idx ;
                k_opt_idx = nan(2,1);
                while FRS_info_idx > 0
                    k_opt_idx = I.k_opt_found(:,FRS_info_idx) ;
                    if ~isnan(k_opt_idx(1))
                        break
                    else
                        FRS_info_idx = FRS_info_idx - 1 ;
                    end
                end
                
                % get the FRS and agent state for the current info index
                if ~isempty(FRS_info_idx) && FRS_info_idx > 0 && ~isnan(k_opt_idx(1))
                    FRS_idx = P.FRS{I.FRS_index(FRS_info_idx)} ;
                    agent_state = I.agent_state(:,FRS_info_idx) ;
                    
                    if check_if_plot_is_available(P,'FRS')
                        % get polynomial sliced by k_opt
                        FRS_poly = msubs(FRS_idx.FRS_polynomial,FRS_idx.k,k_opt_idx) ;
                        
                        % get the 2D contour points to plot
                        [~,FRS_patch_info,N] = get_2D_contour_points(FRS_poly,FRS_idx.z,1,'Bounds',0.9) ;
                        
                        % get the contour with the most vertices
                        [~,plot_idx] = max(N) ;
                        FRS_patch_info = FRS_patch_info(plot_idx) ;
                        
                        % put the vertices in the world frame
                        V = FRS_patch_info.Vertices ;
                        V = FRS_to_world(V',agent_state,...
                            FRS_idx.initial_x,FRS_idx.initial_y,FRS_idx.distance_scale)' ;
                        
                        P.plot_data.FRS.Faces = FRS_patch_info.Faces ;
                        P.plot_data.FRS.Vertices = V ;
                    else
                        if ~isnan(k_opt_idx(1))
                            FRS_data = plot_segway_FRS_in_world_frame(FRS_idx,...
                                k_opt_idx,agent_state,...
                                'FaceColor',[0.5 1 0.3],'FaceAlpha',0.2,...
                                'EdgeColor',[0 0.6 0],'EdgeAlpha',0.5') ;
                            P.plot_data.FRS = FRS_data ;
                        end
                    end
                end
            end
            
            % plot high-level planner
            if P.plot_HLP_flag && info_idx_check
                % plot(P.HLP)
                waypoints = I.waypoints{info_idx} ;
                
                if isempty(waypoints)
                    waypoints = nan(2,1) ;
                end
                
                if check_if_plot_is_available(P,'waypoints')
                    P.plot_data.waypoints.XData = waypoints(1,:) ;
                    P.plot_data.waypoints.YData = waypoints(2,:) ;
                else
                    wps_data = plot_path(waypoints,'--','Color',[0.7 0.5 0.2]) ;
                    P.plot_data.waypoints = wps_data;
                end
            end
            
            if hold_check
                hold off
            end
        end
    end
    
    %% utility methods
    methods (Static)
        function v_0_range = get_v_0_range_from_v_0(v_0,v_0_min,v_0_max)
            % v_0_range = get_v_0_range_from_v_0(v_0)
            % v_0_range = get_v_0_range_from_v_0(v_0,v_0_min,v_0_max)
            %
            % Given a value of v_0, return the v_0 range that contains v_0 (and
            % corresponds to a particular FRS for the turtlebot)
            
            if nargin < 2
                v_0_min = [0.0 0.5 1.0] ;
            end
            
            if nargin < 3
                v_0_max = [0.5 1.0 1.5] ;
            end
            
            % figure out which interval the given v_0 is in
            idxs = (v_0 >= v_0_min) & (v_0 <= v_0_max) ;
            idx = find(idxs,1,'last') ;
            
            % return the corresponding v_0 range
            v_0_range = nan(1,2) ;
            v_0_range(1) = v_0_min(idx) ;
            v_0_range(2) = v_0_max(idx) ;
        end
        
        function FRS = get_FRS_from_v_0(v_0,degree)
            % FRS = get_FRS_from_v_0(v_0)
            %
            % Load the fastest feasible FRS for the given initial speed v_0
            
            if nargin < 2
                degree = 10 ;
            end
            
            v_0_range = get_v_0_range_from_v_0(v_0) ;
            
            switch v_0_range(1)
                case 0.0
                    FRS = load(['segway_FRS_deg_',num2str(degree),'_v_0_0.0_to_0.5.mat']) ;
                case 0.5
                    FRS = load(['segway_FRS_deg_',num2str(degree),'_v_0_0.5_to_1.0.mat']) ;
                case 1.0
                    FRS = load(['segway_FRS_deg_',num2str(degree),'_v_0_1.0_to_1.5.mat']) ;
                otherwise
                    error('Please pick a valid speed range!')
            end
        end
        
        function [r,a] = compute_point_spacings(R,b)
            % [r,a] = compute_point_spacings(R,b)
            %
            % Compute the point spacing r and arc point spacing a, as per Example 67
            % on page 35 of the Big Paper (https://arxiv.org/abs/1809.06746). These are
            % used to discretize obstacles for online planning.
            
            % first, make sure the buffer is valid
            bbar = R ;
            
            if b > bbar
                warning('Resizing obstacle buffer to be valid!')
                b = bbar - 0.01 ;
            end
            
            % now compute the point spacing r and arc point spacing a
            theta_1 = acos((R-b)/R) ;
            theta_2 = acos(b/(2*R)) ;
            r = 2*R*sin(theta_1) ;
            
            if nargout > 1
                a = 2*b*sin(theta_2) ;
            end
        end
        
        function p_str = get_FRS_polynomial_structure(p,z,k,t)
            % p_str = get_FRS_polynomial_structure(p,z,k)
            % p_str = get_FRS_polynomial_structure(p,z,k,t)
            %
            % Given a polynomial p in the variables z and k, decompose it into
            % coefficients and powers, and return an object with this information in a
            % useful format. If p also has terms in t (an optional fourth argument),
            % then the decomposition will include t as well.
            
            % decompose p into matrices and variable IDs
            [vars,pows,coef] = decomp(p) ;
            [~,var_id] = isfree(vars) ;
            
            % iterate through z and get the corresponding columns of the powers
            % matrix, then do the same for k
            Nz = length(z) ;
            z_cols = nan(1,Nz) ;
            for idx = 1:Nz
                [~,z_id] = isfree(z(idx)) ;
                zc = find(var_id == z_id) ;
                z_cols(idx) = zc ;
            end
            
            Nk = length(k) ;
            k_cols = nan(1,Nk) ;
            for idx = 1:Nk
                [~,k_id] = isfree(k(idx)) ;
                kc = find(var_id == k_id) ;
                k_cols(idx) = kc ;
            end
            
            % create output structure
            p_str.pows = pows ;
            p_str.coef = coef ;
            p_str.z_cols = z_cols ;
            p_str.k_cols = k_cols ;
            p_str.t_cols = [] ;
            
            if nargin > 3
                % get the last column as the time index
                [~,t_id] = isfree(t) ;
                t_cols = find(var_id == t_id) ;
                p_str.t_cols = t_cols ;
            end
        end
        
        function [O_FRS, O_buf_out, O_pts_out] = discretize_and_scale_obstacles(O_world,pose,b,r,FRS,O_bounds)
            % O_FRS = discretize_and_scale_obstacles(O_world,pose,b,r,FRS)
            % O_FRS = discretize_and_scale_obstacles(O_world,pose,b,r,FRS,O_bounds)
            %
            % Take obstacle points defined in a global coordinate frame and transform
            % them into the scaled, shifted FRS frame.
            
            % for now, just don't use the arc point spacing by setting the miter limit
            % to 2 in buffer_polygon_obstacles
            O_buf = buffer_polygon_obstacles(O_world,b,2) ;
            
            % handle the world bounds obstacle if it's passed in
            if nargin > 5
                O_bounds_buf = buffer_polygon_obstacles(O_bounds,b,2) ;
                O_buf = [O_buf, nan(2,1), O_bounds_buf] ;
            end
            
            % create discretized obstacle points
            O_pts = interpolate_polyline_with_spacing(O_buf,r) ;
            
            % put the obstacles into the FRS frame
            x0 = FRS.initial_x ;
            y0 = FRS.initial_y ;
            D = FRS.distance_scale ;
            O_FRS = world_to_FRS(O_pts,pose,x0,y0,D) ;
            
            % discard all points behind the robot
            O_log = O_FRS(1,:) >= x0 ;
            O_FRS = O_FRS(:,O_log) ;
            
            % filter out points that are too far away to be reached
            O_FRS = crop_points_outside_region(0,0,O_FRS,1) ;
            
            % return the buffered obstacle as well
            if nargout > 1
                O_buf_out = O_buf ;
                if nargout > 2
                    O_pts_out = O_pts ;
                end
            end
        end
    end
end