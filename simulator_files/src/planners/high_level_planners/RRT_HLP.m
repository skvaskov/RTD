classdef RRT_HLP < high_level_planner
% Class: RRT_HLP < high_level_planner
%
% This implements RRT as a high-level planner for the simulator framework.
% For now, it only works for 2D worlds/obstacles.
%
% The algorithm follows Alg. 1 of this paper:
% http://roboticsproceedings.org/rss06/p34.pdf
%
% Authors: Shreyas Kousik and Bohao Zhang
% Created: 31 July 2019
% Updated: 6 Nov 2019
    
    properties
        % timing and tree growth limits
        timeout = 0.25 ; % seconds
        goal_as_new_node_rate = 0.08 ;
        new_node_growth_distance = 0.5 ; % m
        new_node_heading_change_limit = Inf ; % rad
        new_node_max_distance_from_agent = 2 ; % unused
        best_path = [];
        best_path_indices = 0;
        best_path_distance
        N_nodes ;
        N_nodes_max = 40000 ;
        N_near_goal = 50 ;
        
        % method for growing the RRT:
        % 'new' means grow a new tree every planning iteration
        % 'seed' means use the previous best path to seed a new tree
        % 'keep' means keep growing the same tree every iteration
        grow_tree_mode = 'new' ; % pick 'new' or 'seed' or 'keep'
        
        % tree data structures
        nodes
        nodes_parent
        
        % plot flags
        plot_tree_flag = false ;
        plot_best_path_flag = true ;
        plot_waypoint_flag = false ;
        plot_while_growing_tree_flag = false ; 
    end
    methods
        %% constructor
        function HLP = RRT_HLP(varargin)
            HLP@high_level_planner(varargin{:}) ;
            
            % set up plot data
            HLP.plot_data.tree = [] ;
            HLP.plot_data.best_path = [] ;
            HLP.plot_data.waypoint = [] ;
        end
        
        function setup(HLP,agent_info,world_info)
            setup@high_level_planner(HLP,agent_info,world_info) ;
            HLP.initialize_tree(agent_info) ;
        end
        
        function initialize_tree(HLP,agent_info)
            z = HLP.get_agent_position(agent_info) ;
            
            switch HLP.grow_tree_mode
                case 'new'
                    HLP.vdisp('Growing new tree!',8)
                    HLP.nodes = z ;
                    HLP.nodes_parent = 0 ;
                case 'seed'
                    HLP.vdisp('Seeding from previous best path!',8)
                    HLP.nodes = HLP.best_path ;
                    HLP.nodes_parent = 0:(length(HLP.nodes) - 1) ;
                case 'keep'
                    HLP.vdisp('Keeping previous tree!',8)
                otherwise
                    error('Please pick a valid tree growth mode!')
            end
            
            if isempty(HLP.nodes)
                HLP.nodes = z ;
                HLP.nodes_parent = 0 ;
            end
            
            HLP.N_nodes = size(HLP.nodes,2) ;
        end
        
        function z = get_agent_position(HLP,agent_info)
            z = agent_info.position(:,end) ;
        end
        
        %% tree growth
        function exit_flag = grow_tree(HLP,agent_info,world_info)
            % exit_flag = grow_tree(HLP,agent_info,world_info)
            %
            % Grow the RRT given the current agent and world info. The
            % output exit_flag is 1 if the tree grew at all, and 0
            % otherwise (this happens, for example, if the tree's initial
            % node lies within an obstacle).
            
            % start timing
            start_tic = tic ;
            t_cur = toc(start_tic) ;
            
            % initialize tree
            HLP.initialize_tree(agent_info) ;
            
            % grow tree until timeout or node limit is reached
            while t_cur < HLP.timeout && (HLP.N_nodes < HLP.N_nodes_max)
                % sample
                rand_node = HLP.sample(agent_info) ;
                
                % extend
                extend_success_flag = HLP.extend(agent_info,world_info,rand_node) ;
                
                % plot
                if HLP.plot_while_growing_tree_flag && extend_success_flag
                    T = HLP.make_polyline_tree() ;
                    %HLP.plot_data.tree = plot_path(T,'-','Color',[0.5 0.5 1]) ;
                    plot_object(HLP,T,'tree','Color',[0.7 0.7 1]) ;
                    
                    drawnow()
                end
                
                % increment time
                t_cur = toc(start_tic) ;
            end
            
            % after growing the tree, get path to node closest to goal
            exit_flag = HLP.find_best_path() ;
        end
        
        function rand_node = sample(HLP,agent_info)
            % generate a random node within the bounds of the workspace
            if rand < HLP.goal_as_new_node_rate
                rand_node = HLP.goal ;
            else
                B = HLP.bounds ;
                switch HLP.dimension
                    case 2
                        rand_node = rand_range(B([1 3]),B([2 4]))' ;
                    case 3
                        rand_node = rand_range(B([1 3 5]),B([2 4 6]))' ;
                    otherwise
                        error('The HLP''s dimension must be 2 or 3!')
                end
            end
            
            % make sure the new node is within the agent's sensor horizon
            x_cur = agent_info.position(:,end) ;
            dir_to_node = rand_node - x_cur ;
            dist_to_node = vecnorm(dir_to_node) ;
            R = agent_info.sensor_radius ;
            if dist_to_node > R
                rand_node = x_cur + R.*(dir_to_node ./ dist_to_node) ;
            end
        end
        
        function extend_success_flag = extend(HLP,agent_info,world_info,rand_node)
            [nearest_node,nearest_node_distance,nearest_node_index] = HLP.find_nearest_node(rand_node) ;
            new_node = HLP.steer(rand_node,nearest_node,nearest_node_distance,nearest_node_index) ;
            
            % make sure the new node and nearest node are not the same
            % (this can happen when the RRT is growing near the
            % boundaries of the workspace or near the goal)
            new_node_not_duplicate = (vecnorm(new_node - nearest_node) > 0) && ...
                nearest_node_distance > 0 ;
            
            % check that the new edge is feasible
            if new_node_not_duplicate
                extend_success_flag = HLP.edge_feasibility_check(nearest_node,new_node,world_info) ;
            else
                extend_success_flag = false ;
            end
            
            % add the new node to the tree
            if extend_success_flag
                HLP.add_node_to_tree(new_node,nearest_node_index) ;
            end
        end
        
        function new_node = steer(HLP,rand_node,nearest_node,nearest_node_distance,nearest_node_index)
            % grow the tree towards the direction new node
            NNGD = HLP.new_node_growth_distance ;
            
            % project the random node onto the range of headings allowed
            % from the nearest node
            if (HLP.new_node_heading_change_limit < Inf) && (HLP.dimension == 2) && ...
                    (nearest_node_index > 1)
                % get the parent of the nearest node
                parent_index = HLP.nodes_parent(nearest_node_index) ;
                parent_node = HLP.nodes(:,parent_index) ;
                
                % get the heading to the parent node and to the random node
                node_diff = [nearest_node - parent_node, rand_node - nearest_node] ;
                node_headings = atan2(node_diff(2,:),node_diff(1,:)) ;
                
                % get the change in heading to the random node
                dh = node_headings(1) - node_headings(2) ;
                
                if abs(dh) > HLP.new_node_heading_change_limit
                    % rotate the random node about the nearest node by the
                    % heading delta
                    R = [cos(dh) -sin(dh) ; sin(dh) cos(dh)] ;
                    rand_node = R*(rand_node - nearest_node) + nearest_node ;
                end
            end
            
            
            if nearest_node_distance <= NNGD
                new_node = rand_node ;
            else
                new_node_direction = rand_node - nearest_node ;
                new_node_direction = new_node_direction / norm(new_node_direction) ;
                new_node = nearest_node + NNGD.*new_node_direction ;
            end
        end
        
        function [node,node_dist,node_idx] = find_nearest_node(HLP,node_in)
            % find the nearest node in the tree
            node_distances = vecnorm(HLP.nodes - repmat(node_in,1,HLP.N_nodes)) ;
            [node_dist,node_idx] = min(node_distances) ;
            node = HLP.nodes(:,node_idx) ;
        end
        
        function add_node_to_tree(HLP,new_node,parent_index)
            HLP.nodes = [HLP.nodes, new_node] ;
            HLP.nodes_parent = [HLP.nodes_parent, parent_index] ;
            HLP.N_nodes = HLP.N_nodes + 1 ;
        end
        
        %% path planning
        function exit_flag = find_best_path(HLP)
            % find the node closest to the goal
            distances_to_goal = vecnorm(HLP.nodes - repmat(HLP.goal,1,HLP.N_nodes)) ;
            [~,best_node_index] = min(distances_to_goal) ;
            
            % get best path
            if best_node_index > 1
                HLP.best_path_indices = HLP.tree_path_root_to_node(best_node_index) ;
            else
                HLP.best_path_indices = 1 ;
            end
            
            % get best path
            BP = HLP.nodes(:,HLP.best_path_indices) ;
            
            % make sure it has only unique nodes (this is a sanity check,
            % since sometimes the goal location can get duplicated in the
            % best path)
            HLP.best_path = unique(BP','rows','stable')' ;
            
            % get distance along best path
            HLP.best_path_distance = dist_polyline_cumulative(HLP.best_path) ;
            
            % set waypoints field in case a planner uses it
            HLP.waypoints = HLP.best_path ;
            
            % set up exit flag
            if HLP.best_path_distance(end) > 0
                exit_flag = 1 ;
            else
                exit_flag = 0 ;
            end
        end
        
        function path_indices = tree_path_root_to_node(HLP,destination_index)
            % initialize
            path_indices = destination_index ;
            current_index = destination_index ;
            
            while current_index > 0
                current_parent_index = HLP.nodes_parent(current_index) ;
                if any(path_indices == current_parent_index)
                    HLP.vdisp('Warning! The RRT has a loop in it by accident!',1)
                    break
                end
                
                path_indices = [path_indices, current_parent_index] ;
                current_index = current_parent_index ;
            end
            
            % flip direction
            path_indices = path_indices(end-1:-1:1) ;
        end
        
        function path_indices = tree_path_node_to_node(HLP,source_index,destination_index)
            warning('The tree_path_node_to_node method doesn''t work right yet!')
            
            % get paths from root to source and destination
            path_root_to_source = HLP.tree_path_root_to_node(source_index) ;
            path_root_to_destination = HLP.tree_path_root_to_node(destination_index) ;
            
            % find the last node along both paths
            N_R2S = length(path_root_to_source) ;
            N_R2D = length(path_root_to_destination) ;
            comparison_array = repmat(path_root_to_source(:),1,N_R2D) == ...
                repmat(path_root_to_destination,N_R2S,1) ;
            join_index_R2D = find(any(comparison_array,1),1,'last') ;
            join_index_R2S = find(any(comparison_array,2),1,'last') ;
            
            path_indices = unique([path_root_to_source(end:join_index_R2S), ...
                path_root_to_destination(join_index_R2D:end)],'stable') ;
        end
        
        %% get waypoint
        function wp = get_waypoint(HLP, agent_info, world_info, lookahead_distance)
            % get the agent pose
            z = HLP.get_agent_position(agent_info) ;
            
            % call the RRT star algorithm
            exit_flag = HLP.grow_tree(agent_info, world_info) ;
            
            % if the best path is non-empty...
            if exit_flag
                % get the agent's distance along the best path
                d = dist_point_on_polyline(z,HLP.best_path) ;
                
                % interpolate to get the waypoint
                d_best = HLP.best_path_distance ;
                d_lkhd = min(d_best(end), d + lookahead_distance) ;
                wp = match_trajectories(d_lkhd,d_best,HLP.best_path) ;
            else
                wp = z ;
            end
            
            HLP.current_waypoint = wp ;
        end
        
        %% node feasibility check
        function out = edge_feasibility_check(HLP,node_source,node_target,world_info)
            % this method should return TRUE if the edge is feasible
            O = world_info.obstacles ;
            out = true ; % optimism!
            
            if ~isempty(O)
                X = [node_source, node_target] ;
                [check_inside,~] = inpolygon(X(1,:)',X(2,:)',O(1,:)',O(2,:)') ;
                [check_through,~] = polyxpoly(X(1,:)',X(2,:)',O(1,:)',O(2,:)') ;
                out = isempty(check_through) & ~any(check_inside) ;
            end
        end
        
        %% plotting
        function plot(HLP)
            hc = hold_switch() ;
            
            if HLP.plot_tree_flag
                T = HLP.get_points_for_plot(HLP.make_polyline_tree()) ;
                plot_object(HLP,T,'tree','-','Color',[0.7 0.7 1]) ;
            end
            
            if HLP.plot_best_path_flag
                BP = HLP.get_points_for_plot(HLP.best_path) ;
                plot_object(HLP,BP,'best_path','--',...
                        'Color',[0.7 0.5 0.2],'LineWidth',1.5) ;
            end
            
            if HLP.plot_waypoint_flag
                wp = HLP.get_points_for_plot(HLP.current_waypoint) ;
                plot_object(HLP,wp,'waypoint','kp','LineWidth',1.5) ;
            end
            
            hold_switch(hc) ;
        end
        
        function P = make_polyline_tree(HLP)
            N = HLP.nodes ;
            NP = HLP.nodes_parent ;
            d = HLP.dimension ;
            
            P = [N(:,NP(2:end)) ; N(:,2:end) ; nan(d,size(N,2)-1)] ;
            P = reshape(P,d,[]) ;
        end
        
        function p = get_points_for_plot(HLP,p_in)
            p = p_in ;
            if isempty(p)
                p = nan(2,1) ;
            end
        end
    end
end