classdef segway_grid_HLP < high_level_planner
    properties
        % locations
        start
        goal_index
        best_path
        
        % grid properties
        buffer = 0 ; % meters
        grid_adjacency_matrix = [] ;
        grid_points = [] ; % filled in each time get_waypoint is called
        grid_size = [1 1] ; % [length width]
        grid_spacing = 0.25 ; % distance between points
        grid_cost_increase_factor = 2 ; % cost increase for points being in obs
        grid_cost_decrease_factor = 0.99 ; % cost decrease for points along best path
        grid_cost_max = 1000 ;
        grid_cost_min = 0.001 ;
        
    end
    
    methods
        %% constructor
        function HLP = segway_grid_HLP(varargin)
            HLP = parse_args(HLP,varargin{:}) ;
        end
        
        %% setup
        function setup(HLP,agent_info,world_info)
            HLP.vdisp('Running setup',3)
            
        % 1. get world info
            HLP.vdisp('Getting world info',5)
            % get start and goal
            HLP.start = world_info.start(1:2) ;
            HLP.goal = world_info.goal ;
            
            % get bounds
            b = HLP.buffer ;
            B = world_info.bounds + [b -b b -b] ;
            HLP.bounds = B ;
            
        % 2. make grid of points to cover world
            HLP.vdisp('Making grid',5)
            % make a grid over the world
            N_x_1 = ceil((B(2) - B(1)) ./ HLP.grid_spacing) ;
            N_x_2 = ceil((B(4) - B(3)) ./ HLP.grid_spacing) ;
            x_1_vals = linspace(B(1),B(2),N_x_1) ;
            x_2_vals = linspace(B(3),B(4),N_x_2) ;
            [X_1,X_2] = meshgrid(x_1_vals,x_2_vals) ;
            X = [X_1(:) X_2(:)]' ;
            
            % make sure the start and goal are in the grid
            N_x = size(X,2) ;
            if all(vecnorm(X - repmat(HLP.start,1,N_x)) > 0)
                HLP.vdisp('Adding start to grid',6)
                X = [HLP.start, X] ;
                N_x = N_x + 1 ;
            end
            
            if all(vecnorm(X - repmat(HLP.goal,1,N_x)) > 0)
                HLP.vdisp('Adding goal to grid',6)
                X = [X, HLP.goal] ;
                N_x = N_x + 1 ;
            end
            
            HLP.grid_points = X ;
            [~,HLP.goal_index] = min(vecnorm(X - repmat(HLP.goal,1,N_x))) ;
            
        % 3. connect grid as an adjacency matrix (i.e., graph)
            HLP.vdisp('Making adjacency matrix',5)
            adj_matrix_start_tic = tic ;
            % connect all points that are close enough to each other
            d_connect = 1.1*sqrt(2)*HLP.grid_spacing ;
            [idx,D] = knnsearch(X',X','K',9) ;
            idx = idx .* (D <= d_connect) ; % each row contains the distance to connected neighbors
            
            % make adjacency matrix
            A = sparse(N_x,N_x) ;
            
            % iterate through the rows of idx to find (x_1,x_2) pairs that
            % should be connected in A
            for x_1_idx = 1:size(idx,1)
                % get the adjacent nodes
                x_2_idx_log = idx(x_1_idx,:) ~= 0 ;
                x_2_idx = idx(x_1_idx,x_2_idx_log) ;
                
                % put the corresponding distances into A
                d_idx = abs(D(x_1_idx,x_2_idx_log)) ;
                A(x_1_idx,x_2_idx) = d_idx ;
                A(x_2_idx,x_1_idx) = d_idx ;
            end
            HLP.grid_adjacency_matrix = A ;
            adj_matrix_time = toc(adj_matrix_start_tic) ;
            HLP.vdisp(['Adjacency matrix took ',...
                num2str(adj_matrix_time,'%0.2f'),' s to create'],4)
        end
        
        %% get waypoint
        function waypoint = get_waypoint(HLP,agent_info,world_info,lookahead_distance)
            try
                O_str = world_info.obstacles_struct ;
            catch
                error(['The segway_grid_HLP requires an obstacle structure ',...
                    'to be passed in as world_info.obstacle_struct! This ',...
                    'structure must have the fields A, b, N_obs, and ',...
                    'N_halfplanes, which are the outputs of the ',...
                    'segway_box_obstacles_to_halfplanes function'])
            end
            
            % get the grid points and adjacency matrix
            X = HLP.grid_points ;
            A = HLP.grid_adjacency_matrix ;
            
            % check which points are inside the obstacles
            X_obs_log = check_points_in_halfplane_obstacles(X,O_str) ;
            
            % turn the points inside into a logical array of all edges in
            % the grid adjacency matrix that are connected to points in
            % obstacles
            X_obs_log_mat = repmat(X_obs_log,length(X_obs_log),1) ;
            X_obs_log_mat = X_obs_log_mat | (X_obs_log_mat') ;
            
            % increase the cost of the points inside obstacles
            weight_log_mat = (A .* X_obs_log_mat) > 0 ;
            A(weight_log_mat) = A(weight_log_mat).* HLP.grid_cost_increase_factor ;
            
            % cap the cost in A
            A(A > HLP.grid_cost_max) = HLP.grid_cost_max ;
            
            % find the point nearest to the agent
            N_x = size(X,2) ;
            [~,close_index] = min(vecnorm(X - repmat(agent_info.position(:,end),1,N_x))) ;
            
            % get the best path from the agent to the goal
            [~,best_path_indices,~] = graphshortestpath(A,close_index,HLP.goal_index) ;
            HLP.waypoints = X(:,best_path_indices) ;
            waypoint_distance = dist_polyline_cumulative(HLP.waypoints) ;
            
            % interpolate along the best path to get the waypoint
            lookahead_distance = min([lookahead_distance, waypoint_distance(end)]) ;
            waypoint = match_trajectories(lookahead_distance,waypoint_distance,HLP.waypoints) ;
            HLP.current_waypoint = waypoint ;
            
            % reduce the cost of all points along the best path
            [r,c] = size(A) ;
            B = logical(sparse(r,c)) ;
            p_0 = best_path_indices(1:(end-1)) ;
            p_1 = best_path_indices(2:end) ;
            B(p_0,p_1) = true ;
            B(p_1,p_0) = true ;
            A(B) = HLP.grid_cost_decrease_factor.*A(B) ;
            
            % lower-bound the cost in A
            A((A ~= 0) & (A < HLP.grid_cost_min)) = HLP.grid_cost_min ;
            
            % put the adjacency matrix back
            HLP.grid_adjacency_matrix = A ;
        end
    end
end