classdef static_box_world < world
    properties
        buffer = 1 ;
        obstacles_unseen
        obstacles_seen
        obstacle_size_bounds = [0.3,0.7] ;
        obstacle_rotation_bounds = [-pi,pi] ;
        bounds_as_polyline
        
        % plotting
        obstacle_seen_color = [1 0 0] ;
        obstacle_unseen_color = [1 0.6 0.6] ;
    end
    
    methods
        %% constructor
        function W = static_box_world(varargin)
            % set default properties
            bounds = [-7 7 -4 4] ;
            N_obstacles = 0 ;
            
            % parse other args
            W = parse_args(W,'bounds',bounds,'N_obstacles',N_obstacles,varargin{:}) ;
            
            % call setup to generate obstacles
            W.setup() ;
            
            % check for mapping toolbox
            try
                polyxpoly([],[],[],[]) ;
            catch
                error(['Please make sure the Mapping Toolbox is installed',...
                    ' so you can use polyxpoly.'])
            end
        end
        
        %% setup
        function setup(W)
            W.vdisp('Setting up world',3)
            
            % get room bounds
            B = W.bounds ;
            xlo = B(1) ; xhi = B(2) ; ylo = B(3) ; yhi = B(4) ;
            W.bounds_as_polyline = make_box([xhi-xlo,yhi-ylo]) + repmat(mean([xhi xlo ; yhi ylo],2),1,5) ;
            
            % get obstacle info
            obs_size = W.obstacle_size_bounds ;
            obs_rotation_bounds = W.obstacle_rotation_bounds ;
            
            % generate start position on left side of room with initial
            % heading of 0, and make sure it's not too close to the walls
            b = W.buffer ;
            
            xlo = xlo + 2*b ;
            xhi = xhi - 2*b ;
            ylo = ylo + 2*b ;
            yhi = yhi - 2*b ;
            
            if isempty(W.start)
                W.vdisp('Generating random start pose',5)
                s = [xlo ;
                    rand_range(ylo, yhi) ;
                    0 ] ;
                W.start = s ;   
            end
            
            % generate goal position on right side of room
            if isempty(W.goal)
                W.vdisp('Generating random goal location',5)
                g = [xhi ;
                    rand_range(ylo, yhi)] ;
                W.goal = g ;
            end
            
            % generate obstacles around room
            N_obs = W.N_obstacles ;
            
            if N_obs > 0 && isempty(W.obstacles)
                W.vdisp('Generating obstacles',4)
                O = nan(2, 6*N_obs) ; % preallocate obstacle matrix
                
                llo = obs_size(1) ; lhi = obs_size(2) ;
                orlo = obs_rotation_bounds(1) ;
                orhi = obs_rotation_bounds(2) ;
                
                xlo = B(1) ; xhi = B(2) ; ylo = B(3) ; yhi = B(4) ;
                xlo = xlo + b ; xhi = xhi - b ;
                ylo = ylo + b ; yhi = yhi - b ;
                
                for idx = 1:6:(6*N_obs-1)
                    l = (lhi-llo)*rand(1) + llo ; % length
                    r = (orhi-orlo)*rand(1) + orlo ; % rotation
                    
                    % obstacle rotation
                    R = [cos(r) sin(r) ; -sin(r) cos(r)] ;
                    
                    % obstacle base
                    o = [-l/2  l/2 l/2 -l/2 -l/2 ;
                        -l/2 -l/2 l/2  l/2 -l/2 ] ;
                    
                    % obstacle center, which must not be too close to the start
                    % or goal
                    d_center = 0 ;
                    ds_limit = 0.1*max((xhi-xlo),(yhi-ylo)) ;
                    dg_limit = W.goal_radius ;
                    while (d_center < ds_limit || d_center < dg_limit)
                        c = [(xhi-xlo)*rand(1) + xlo ;
                            (yhi-ylo)*rand(1) + ylo ] ;
                        ds = min(dist_point_to_points(s(1:2),c)) ;
                        dg = min(dist_point_to_points(g,c)) ;
                        d_center = min(ds,dg) ;
                    end
                    
                    O(:,idx:idx+4) = R*o + repmat(c,1,5) ;
                end
                
                if isnan(O(1,end))
                    O = O(:,1:end-1) ;
                end
                
                W.obstacles = O ;
                W.obstacles_seen = [] ;
                W.N_obstacles = N_obs ;
            end
            
            W.obstacles_unseen = W.obstacles ;
            
            % set up plot data
            W.plot_data.obstacles_seen = [] ;
            W.plot_data.obstacles_unseen = [] ;
            W.plot_data.start = [] ;
            W.plot_data.goal = [] ;
            W.plot_data.goal_zone = [] ;
            W.plot_data.bounds = [] ;
        end
        
        %% get obstacles
        function world_info = get_world_info(W,agent_info,~)
            W.vdisp('Getting world info!',3)
            
            if nargin > 1
                zcur = agent_info.state(agent_info.position_indices,end)' ;
                zcur = round(zcur,6) ;
                z = unique(zcur,'rows')' ;
                r = agent_info.sensor_radius ;
            else
                W.vdisp('No agent info provided!',2)
                z = W.start ;
                r = 1 ;
            end
            
            O = W.obstacles_unseen ;
            
            if ~isempty(O)
                N = size(O,2)/6 ; % number of obstacles still unseen
                
                indices_seen = [] ;
                
                O_out = [] ;
                for idx = 1:6:(6*N-1)
                    if size(z,2) == 1
                        dToObs = dist_point_to_polyline(z,O(:,idx:idx+4)) ;
                    else
                        dToObs = poly_poly_dist(z(1,:),z(2,:),...
                            O(1,idx:idx+4),O(2,idx:idx+4)) ;
                    end
                    if any(dToObs <= r)
                        O_out = [O_out, O(:,idx:idx+4),nan(2,1)] ;
                        indices_seen = [indices_seen, idx:idx+5] ;
                    end
                    
                    % make sure the last index seen is within the length of O
                    if ~isempty(indices_seen) && indices_seen(end) > size(O,2)
                        indices_seen = indices_seen(1:end-1) ;
                    end
                end
                
                % if some obstacles were seen, remove them from O
                O(:,indices_seen) = [] ;
                W.obstacles_unseen = O ;
                W.obstacles_seen = [W.obstacles_seen, O_out] ;
            end
            
            world_info.obstacles = W.obstacles_seen ;
            world_info.bounds = W.bounds ;
            world_info.start = W.start ;
            world_info.goal = W.goal ;
            world_info.dimension = W.dimension ;
        end
        
        
        %% collision check
        function out = collision_check(W,agent_info,check_full_traj_flag)
            
            % by default, don't check the full trajectory
            if nargin < 3
                check_full_traj_flag = false ;
            end
            
            % initialize output (innocent until proven guilty)
            out = 0 ;
            
            % extract agent info
            pos_idx = agent_info.position_indices ;
            h_idx = agent_info.heading_index ;
            fp = agent_info.footprint_vertices ;
            Z = agent_info.state ;
            T = agent_info.time ;
            
            % set up obstacles
            O = [W.obstacles, nan(2,1), W.bounds_as_polyline] ;
            
            % if the agent has moved, we need to check if it crashed;
            % alternatively, we could check the entire trajectory for crashes;
            % finally, if it didn't move, then check the full trajectory
            t_start = W.current_time ;
            if check_full_traj_flag
                t_log = true(size(T)) ;
            else
                t_log = T >= t_start ;
            end
            
            if sum(t_log) > 1
                T = T(t_log) ;
                Z = Z(:,t_log) ;
                
                % create the desired time vector; check for collision every
                % 10ms by default, in 5s chunks of time
                dt = T(end) - T(1) ; % total time of traj
                N_chk = ceil(dt/5) ; % number of checks to make
                
                t_world = T(1) ; % start time of each check
                
                for chk_idx = 1:N_chk
                    % get the time vector to check
                    t_log_idx = T >= t_world ;
                    T_idx = T(t_log_idx) ;
                    Z_idx = Z(:,t_log_idx) ;
                    
                    % create the time vector for interpolation
                    t_chk = (0:0.01:5) + t_world ;
                    t_chk = t_chk(t_chk <= T_idx(end)) ;
                    
                    % create the interpolated trajectory
                    try
                        Z_chk = match_trajectories(t_chk,T_idx,Z_idx) ;
                        
                        % create a copy of the agent footprint, rotated at each
                        % point of the trajectory
                        X = Z_chk(pos_idx,:) ; % xy positions of trajectory
                        N = size(X,2) ;
                        X = repmat(X(:),1,size(fp,2)) ;
                        
                        % X is rep'd for each pt of agent footprint vertices
                        
                        if ~isempty(h_idx) && (length(agent_info.footprint) > 1)
                            % if there is a heading, and the agent is not a circle,
                            % then rotate each footprint contour
                            H = Z_chk(h_idx,:) ;
                            R = rotation_matrix_2D(H) ;
                            F = R*repmat(fp,N,1) + X ;
                        else
                            % otherwise, just place the footprint contour at each
                            % point of the trajectory
                            F = repmat(fp,N,1) + X ;
                        end
                        
                        Fx = [F(1:2:end,:)' ; nan(1,N)] ;
                        Fy = [F(2:2:end,:)' ; nan(1,N)] ;
                        F = [Fx(:)' ; Fy(:)'] ;
                        
                        % check if the resulting contour intersects the obstacles
                        [ci,~] = polyxpoly(F(1,:),F(2,:),O(1,:),O(2,:)) ;
                        
                        if ~isempty(ci)
                            out = 1 ;
                            break
                        end
                        
                        % increment the time index
                        t_world = t_world + 5 ;
                    catch
                        W.vdisp('Check failed, skipping to next portion!',2)
                        out = -1 ;
                    end
                end
            end
            
            % update the world time index
            W.current_time = agent_info.time(end) ;
        end
        
        %% plotting
        function plot(W,~)
            % set up hold if needed
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on
            end
            
            % plot sensed obstacles
            O_seen = W.obstacles_seen ;
            
            if isempty(O_seen)
                O_seen = nan(2,1) ;
            end
            
            if check_if_plot_is_available(W,'obstacles_seen')
                W.plot_data.obstacles_seen.XData = O_seen(1,:) ;
                W.plot_data.obstacles_seen.YData = O_seen(2,:) ;
            else
                seen_data = plot(O_seen(1,:),O_seen(2,:),'Color',W.obstacle_seen_color) ;
                W.plot_data.obstacles_seen = seen_data ;
            end
            
            % plot unsensed obstacles
            O_unseen = W.obstacles_unseen ;
            
            if isempty(O_unseen)
                O_unseen = nan(2,1) ;
            end
            
            if check_if_plot_is_available(W,'obstacles_unseen')
                W.plot_data.obstacles_unseen.XData = O_unseen(1,:) ;
                W.plot_data.obstacles_unseen.YData = O_unseen(2,:) ;
            else
                unseen_data = plot(O_unseen(1,:),O_unseen(2,:),'Color',W.obstacle_unseen_color) ;
                W.plot_data.obstacles_unseen = unseen_data ;
            end
            
            % plot start
            s = W.start ;
            
            if ~check_if_plot_is_available(W,'start')
                start_data = plot(s(1),s(2),'bx') ;
                W.plot_data.start = start_data ;
            end
            
            % plot goal and goal zone
            g = W.goal ;
            g_circ = make_circle(W.goal_radius,100) + repmat(g(1:2),1,100) ;
            
            if ~check_if_plot_is_available(W,'goal')
                goal_data = plot(g(1),g(2),'x','Color',[0.2 0.7 0]) ;
                W.plot_data.goal = goal_data ;
            end
            
            if ~check_if_plot_is_available(W,'goal_zone')
                goal_zone_data = plot(g_circ(1,:),g_circ(2,:),'--','Color',[0.2 0.7 0]) ;
                W.plot_data.goal_zone = goal_zone_data ;
            end
            
            % plot bounds
            B = W.bounds_as_polyline ;
            
            if ~check_if_plot_is_available(W,'bounds')
                bounds_data = plot(B(1,:),B(2,:),'Color',W.obstacle_seen_color) ;
                W.plot_data.bounds = bounds_data ;
            end
            
            if hold_check
                hold off ;
            end
        end
        
        function plot_at_time(W,~)
            % set up hold if needed
            hold_check = hold_switch() ;
            
            % plot obstacles
            O = W.obstacles ;
            
            if isempty(O)
                O = nan(2,1) ;
            end
            
            if check_if_plot_is_available(W,'obstacles')
                W.plot_data.obstacles.XData = O(1,:) ;
                W.plot_data.obstacles.YData = O(2,:) ;
            else
                seen_data = plot(O(1,:),O(2,:),'Color',W.obstacle_seen_color) ;
                W.plot_data.obstacles = seen_data ;
            end
            
            % plot start
            s = W.start ;
            
            if ~check_if_plot_is_available(W,'start')
                start_data = plot(s(1),s(2),'bx') ;
                W.plot_data.start = start_data ;
            end
            
            % plot goal and goal zone
            g = W.goal ;
            g_circ = make_circle(W.goal_radius,100) + repmat(g(1:2),1,100) ;
            
            if ~check_if_plot_is_available(W,'goal')
                goal_data = plot(g(1),g(2),'x','Color',[0.2 0.7 0]) ;
                W.plot_data.goal = goal_data ;
            end
            
            if ~check_if_plot_is_available(W,'goal_zone')
                goal_zone_data = plot(g_circ(1,:),g_circ(2,:),'--','Color',[0.2 0.7 0]) ;
                W.plot_data.goal_zone = goal_zone_data ;
            end
            
            % plot bounds
            B = W.bounds_as_polyline ;
            
            if ~check_if_plot_is_available(W,'bounds')
                bounds_data = plot(B(1,:),B(2,:),'Color',W.obstacle_seen_color) ;
                W.plot_data.bounds = bounds_data ;
            end
            
            % set axes
            axis equal
            
            hold_switch(hold_check) ;
        end
    end
end