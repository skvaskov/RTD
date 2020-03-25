classdef segway_generic_planner < planner
    %%
    properties
        % agent information
        agent_footprint = 0.38 ;
        agent_max_speed = 1.5 ; % m/s
        agent_max_accel = 3 ;
        agent_max_yaw_rate = 1 ;
        agent_max_yaw_accel = 5.9 ;
        agent_average_speed = inf ; % over P.agent_average_speed_time_horizon
        agent_average_speed_time_horizon = 1 ; % s
        agent_average_speed_threshold = 1e-3 ; % m/s
        
        % plan handling
        current_waypoint
        lookahead_distance = 1.25 ; % meters
        buffer_for_HLP = 0.1 ;
        
        % plotting
        plot_obstacles_flag = true ;
        plot_waypoints_flag = false ;
    end
    
    %%
    methods
        %% constructor
        function P = segway_generic_planner(varargin)
            name = 'Segway Planner' ;
            HLP = straight_line_HLP() ;
            P@planner('name',name,'HLP',HLP,varargin{:}) ;
        end
        
        %% setup
        function setup(P,agent_info,world_info)
        % 1. set up info from agent and world
            P.agent_max_accel = agent_info.max_accel ;
            P.agent_max_yaw_rate = agent_info.max_yaw_rate ;
            b = P.buffer ;
            P.bounds = world_info.bounds + [b -b b -b] ;
            
        % 2. set up high-level planner
            P.HLP.setup(agent_info,world_info) ;
            P.HLP.default_lookahead_distance = P.lookahead_distance ;

        % 3. initialize current plan
            P.current_plan.T = [] ;
            P.current_plan.U = [] ;
            P.current_plan.Z = [] ;
            
        % 4. set up info structure
            P.info = struct('agent_time',[],'agent_state',[],...
                'waypoint',[],...
                'waypoints',[],...
                'obstacles',[],...
                'T',[],'U',[],'Z',[]) ;
        end
        
        %% process obstacles
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
        
        %% increment plan
        function [T,U,Z,stop_flag] = increment_plan(P,agent_info)
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
                P.vdisp('Incrementing previous plan!',6)
                
                % make sure P.t_move exists in the trajectory
                T_temp = [T_old(T_old < P.t_move), P.t_move, T_old(T_old > P.t_move)] ;
                [U,Z] = match_trajectories(T_temp,T_old,U_old,T_old,Z_old) ;
                T_log = T_temp >= P.t_move ;
                
                % increment the time and input
                T = T_temp(T_log) - P.t_move ;
                U = U(:,T_log) ;
                Z = Z(:,T_log) ;
                
                stop_flag = false ;
            else
                P.vdisp('Making stopped plan!',6)
                T = 0 ;
                U = zeros(2,1) ;
                Z = [agent_info.state(1:3,end) ; zeros(2,1)] ;
                
                stop_flag = true ;
            end
        end
        
        %% make yaw rate for spinning in place
        function w_des = make_yaw_rate_towards_waypoint(P,z_cur,z_goal)
            % get the waypoint in local coordinates
            z_loc = world_to_local(z_cur,z_goal) ;
            
            % get the heading to the local waypoint
            h_loc = atan2(z_loc(2),z_loc(1)) ;
            
            % create a yaw rate to turn towards the local waypoint
            w_des = P.agent_max_yaw_rate*sign(h_loc)*rand(1) ;
        end
        
        %% create plan when traj opt failed
        function [T,U,Z] = make_plan_for_traj_opt_failure(P,agent_info)
                P.vdisp('Continuing previous plan!',5)
                
                [T,U,Z,~] = P.increment_plan(agent_info) ;
                
                % make a spin-in-place maneuver of duration P.t_move
                P.vdisp('Adding spin maneuver to previous plan!',7)
                
                % w_cur = agent_info.state(agent_info.yaw_rate_index,end) ;
                % delta_w = FRS_cur.delta_w ;
                % w_des = make_segway_random_yaw_rate(w_cur,delta_w,P.agent_max_yaw_rate) ;
                
                w_des = P.make_yaw_rate_towards_waypoint(agent_info.state(:,end),P.current_waypoint) ;
                [T_spin,U_spin,Z_spin] = make_segway_spin_trajectory(P.t_move,w_des) ;
                
                T_spin = T_spin + T(end) ;
                
                % rotate and shift Z_spin to the end of the agent's current
                % trajectory
                Z_spin(1:3,:) = repmat(agent_info.state(1:3,end),1,size(Z_spin,2)) ;
                
                % add the spin maneuver to the end of the previous
                % trajectory
                T = [T(1:end-1), T_spin] ;
                U = [U(:,1:end-1), U_spin] ;
                Z = [Z(:,1:end-1), Z_spin] ;
                
                % if the new plan is not long enough, add stopping maneuver
                % to the end of it
                if T(end) < P.t_move
                    T = [T, P.t_move, 2*P.t_move] ;
                    U = [U, zeros(2,1)] ;
                    Z = [Z, [Z(1:3,end) ; 0; 0], [Z(1:3,end) ; 0; 0] ] ;
                end
        end
        
        %% update info object
        function update_info(P,agent_info, waypoint, O, T, U, Z)
            I = P.info ;
            
            I.agent_time = [I.agent_time, agent_info.time(end)] ;
            I.agent_state = [I.agent_state, agent_info.state(:,end)] ;
            I.waypoint = [I.waypoint, waypoint] ;
            I.waypoints = [I.waypoints, {P.HLP.waypoints}] ;
            I.obstacles = [I.obstacles, {O}] ;
            I.T = [I.T, {T}] ;
            I.U = [I.U, {U}] ;
            I.Z = [I.Z, {Z}] ;
            
            P.info = I ;
        end
        
        %% plot
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
        
        function plot_at_time(P,t)
             if nargin < 2
                if ~isempty(P.info.agent_time)
                    t = P.info.agent_time(end) ;
                else
                    t = 0 ;
                end
            end
            
            hold_check = hold_switch() ;
            
            % figure out the info index closest to the current time
            I = P.info ;
            info_idx = find(t >= I.agent_time,1,'last') ;
            info_idx_check = ~isempty(info_idx) ;
            
            if info_idx_check
                % plot current obstacles
                if P.plot_obstacles_flag
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
                if P.plot_waypoints_flag
                    wp = I.waypoint(:,info_idx) ;
                    if isempty(wp)
                        wp = nan(2,1) ;
                    end
                    
                    if check_if_plot_is_available(P,'waypoint')
                        P.plot_data.waypoint.XData = wp(1) ;
                        P.plot_data.waypoint.YData = wp(2) ;
                    else
                        wp_data = plot(wp(1),wp(2),'bp') ;
                        P.plot_data.waypoint = wp_data ;
                    end
                end
                
                % plot high-level planner
                if P.plot_HLP_flag
                    plot(P.HLP)
                end
            end
            
            hold_switch(hold_check) ;
        end
    end
end