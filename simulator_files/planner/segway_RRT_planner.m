classdef segway_RRT_planner < segway_generic_planner
% Class: segway_RRT_planner < segway_generic_planner
%
% This implements an RRT trajectory planner for the segway, based roughly
% on Yoshiaki Kuwata's work on planning for the MIT DARPA Urban Challenge.
% It also borrows some code from RRT_HLP.m in the simulator repo.
%
% Author: Shreyas Kousik
% Created: 2018
% Updated: 23 March 2020
    
    %% properties
    properties
        nodes = nan(5,10000) ;
        edges = nan(2,10000) ;
        depth = zeros(1,10000) ;
        N_nodes = 0 ;
        N_nodes_max = 10000 ;
        nodes_parent = zeros(1,10000);
        best_traj_indices = [] ;
        grow_tree_mode = 'new' ; % pick 'new' or 'seed' or 'keep'
        delta_w = 0.25 ;
        delta_v = 0.125 ;
        waypoint_reached_distance = 0.1 ; % m
        
        % timing for each small trajectory (i.e., edge) grown in the RRT
        dt_edge = 0.01 ;
        t_edge = 0.25 ;
    end
    
    %% methods
    methods
        %% constructor
        function P = segway_RRT_planner(varargin)
            name = 'Segway RRT Planner' ;
            P@segway_generic_planner('name',name,varargin{:}) ;
        end
        
        %% setup
        function setup(P,agent_info,world_info)
        % 1. call superclass method
            setup@segway_generic_planner(P,agent_info,world_info) ;
            
        % 2. initialize tree
            P.initialize_tree(agent_info) ;
            
        % 3. load symbolic traj data
            D = load('segway_symbolic_traj_data.mat') ;
            P.dt_edge = D.dt_traj ;
            P.t_edge = D.t_traj ;
            
        % 4. set delta_w and delta_v
            P.delta_w = P.agent_max_yaw_accel * P.t_edge ;
            P.delta_v = P.agent_max_accel * P.t_edge ;
        end
        
        function initialize_tree(P,agent_info)
            % try to initialize the tree from the previous plan
            if ~isempty(P.current_plan.T)
                z = match_trajectories(P.t_move,P.current_plan.T,P.current_plan.Z) ;
            else
                z = nan ;
            end
            
            if any(isnan(z))
                P.vdisp('Using agent state to initialize RRT',5) ;
                z = agent_info.state(:,end) ;
            else
                P.vdisp('Using previous trajectory to initialize RRT',5) ;
            end

            switch P.grow_tree_mode
                case 'new'
                    P.vdisp('Growing new tree!',8)
                    P.nodes = z ;
                    P.nodes_parent = 0 ;
                    P.edges = zeros(2,1) ;
                    P.depth = 0 ;
                case 'seed'
                    P.vdisp('Seeding from previous best path!',8)
                    P.nodes = P.nodes(:,P.best_traj_indices) ;
                    P.nodes_parent = 0:(length(P.nodes) - 1) ;
                    P.edges = P.edges(:,P.best_traj_indices) ;
                    P.depth = 0:(length(P.nodes) - 1) ;
                case 'keep'
                    P.vdisp('Keeping previous tree!',8)
                otherwise
                    error('Please pick a valid tree growth mode!')
            end
            
            if isempty(P.nodes)
                P.vdisp('Growing new tree!',8)
                P.nodes = z ;
                P.nodes_parent = 0 ;
                P.edges = zeros(2,1) ;
                P.depth = 0 ;
            end
            
            P.N_nodes = size(P.nodes,2) ;
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
        % 0. start timing
            start_tic = tic ;
            
        % 1. get agent info
            P.agent_average_speed = get_segway_average_speed(agent_info.time,...
                agent_info.state(agent_info.speed_index,:),...
                P.agent_average_speed_time_horizon) ;
        
        % 2. set up obstacles
            [O_buf,A_obs,b_obs,N_obs,N_hfp] = P.process_obstacles(world_info) ;
        
        % 3. get waypoint to grow RRT towards
            lkhd = (P.agent_average_speed + P.lookahead_distance) / 2 ;
            world_info.obstacles = O_buf ;
            z_goal = P.HLP.get_waypoint(agent_info,world_info,lkhd) ;
            P.current_waypoint = z_goal ;
            
        % 3. grow the RRT
            P.vdisp('Growing RRT',4)
            t_cur = toc(start_tic) ;
            P.initialize_tree(agent_info) ;
            
            % make sure we're not planning from inside any obstacles
            collision_node_log = P.collision_check(P.nodes(1:2,:),A_obs,b_obs,N_hfp,N_obs) ;
            if any(~collision_node_log)
                disp('hey!')
            end
            
            waypoint_not_reached_flag = true ;
            
            while t_cur < P.t_plan && (P.N_nodes <= P.N_nodes_max) && ...
                    waypoint_not_reached_flag
                
                % pick a random node
                idx_rand = rand_int(1,P.N_nodes) ;
                z_rand = P.nodes(:,idx_rand) ;
                
                % select a random control input that is feasible given that
                % node's velocity and yaw rate
                u_rand = P.sample_control_inputs(z_rand) ;
               
                % forward-integrate the dynamics from the random node with
                % the random control input, using the symbolic function
                % created by generate_segway_symbolic_trajectory.m
                Z_new = segway_symbolic_traj(z_rand(4),z_rand(5),u_rand(1),u_rand(2)) ;
                
                % shift/rotate new trajectory to the location of z_rand
                Z_new = local_to_world(z_rand,Z_new) ;
                
                % get position trajectory for collision checking
                X_new = Z_new(1:2,:) ;
                
                % make sure the new trajectory is in bounds
                B_chk = (X_new(1,:) >= P.bounds(1)) & (X_new(1,:) <= P.bounds(2)) &...
                    (X_new(2,:) >= P.bounds(3)) & (X_new(2,:) <= P.bounds(4)) ;
                B_chk = all(B_chk) ;
                
                if B_chk && all(P.collision_check(X_new,A_obs,b_obs,N_hfp,N_obs))
                    % update the tree
                    N = P.N_nodes + 1 ;
                    P.nodes(:,N) = Z_new(:,end) ;
                    P.nodes_parent(N) = idx_rand ;
                    P.edges(:,N) = u_rand ;
                    P.depth(N) = P.depth(idx_rand) + 1 ;
                    P.N_nodes = N ;
                    
                    % check if the waypoint has been reached
                    d_to_wp = vecnorm(z_goal(1:2) - X_new(:,end)) ;
                    waypoint_not_reached_flag = d_to_wp > P.waypoint_reached_distance ;
                    
                    % % FOR DEBUGGING
                    % plot_path(Z_new,'b-') ;
                end
                
                % timing
                t_cur = toc(start_tic) ;
            end
            
        % 4. get the best path from the tree
        [traj_indices,exit_flag] = P.find_best_path() ;
        
        % 5. process RRT result
        [T,U,Z] = process_traj_opt_result(P,traj_indices,exit_flag,agent_info) ;
        
        % % FOR DEBUGGING
        % figure(2) ; clf ;
        % plot(T,Z')
        
        % 6. update current plan and info
            P.current_plan.T = T ;
            P.current_plan.U = U ;
            P.current_plan.Z = Z ;
            P.update_info(agent_info,z_goal,O_buf,T,U,Z) ;
        end
        
        %% replan: sample random yaw rate and speed
        function u = sample_control_inputs(P,z)
            % u = sample_control_inputs(P,z)
            %
            % Create a random control input from the given state
            
            % create yaw rate
            w_cur = z(4) ;
            w_des_lo = max(w_cur - P.delta_w, -P.agent_max_yaw_rate) ;
            w_des_hi = min(w_cur + P.delta_w, +P.agent_max_yaw_rate) ;
            w_mean = w_cur ;
            w_std = P.agent_max_yaw_rate ;
            w_des = rand_range(w_des_lo,w_des_hi,w_mean,w_std) ;

            % create speed
            v_cur = z(5) ;
            v_des_lo = max(v_cur - P.delta_v, 0) ;
            v_des_hi = min(v_cur + P.delta_v, P.agent_max_speed) ;
            v_mean = 3*P.agent_max_speed/4 ;
            v_std = P.agent_max_speed/2 ;
            v_des = rand_range(v_des_lo,v_des_hi,v_mean,v_std) ;
            
            % return output
            u = [w_des ; v_des] ;
        end
        
        %% replan: collision check
        function out = collision_check(P,X,A_obs,b_obs,N_hfp,N_obs)
            % return TRUE if NOT in collision
            out = A_obs*X - b_obs ;
            out = reshape(out,N_hfp,[]) ;
            out = reshape(max(out,[],1),N_obs,[]) ;
            out = (-min(out,[],1)) < 0 ;
        end
        
        %% replan: find best path
        function [traj_indices,exit_flag] = find_best_path(P)
            % initialize path and exit flag pessimistically
            traj_indices = 1 ;
            exit_flag = 0 ;
            
            % get closest node to waypoint
            distances_to_goal = vecnorm(P.nodes(1:2,:) - repmat(P.current_waypoint(1:2),1,P.N_nodes)) ;
            [~,best_node_index] = min(distances_to_goal) ;
            
            % get best path
            if best_node_index > 1
                % initialize
                traj_indices = best_node_index ;
                current_index = best_node_index ;
                
                % iterate backwards until reaching the tree root
                while current_index > 0
                    current_parent_index = P.nodes_parent(current_index) ;
                    
                    if any(traj_indices == current_parent_index)
                        warning('Warning! The RRT has a loop in it by accident!')
                        exit_flag = -1 ;
                        break
                    end
                    
                    traj_indices = [traj_indices, current_parent_index] ;
                    current_index = current_parent_index ;
                end
                
                % flip direction
                traj_indices = traj_indices(end-1:-1:1) ;
                exit_flag = 1 ;
            end
        end
        
        %% replan: process RRT output
        function [T,U,Z] = process_traj_opt_result(P,traj_indices,exit_flag,agent_info)
            P.vdisp('Creating plan from RRT result',4)
            
            if exit_flag > 0
                P.vdisp('New plan successfully found!',5) ;
                P.best_traj_indices = traj_indices ;
                
                % get the control inputs used to generate the new plan
                % (note, we don't need the first one since it's always
                % zeros, starting from the root node)
                U_temp = P.edges(:,traj_indices(2:end)) ;
                N_U = size(U_temp,2) ;
                
                % generate the trajectory
                U = U_temp(:,1) ;
                Z = P.nodes(:,1) ;
                
                for idx = 1:N_U
                    % get  w_0, v_0, w_des, and v_des
                    z = Z(:,end) ;
                    w_0 = z(4) ;
                    v_0 = z(5) ;
                    
                    u = U_temp(:,idx) ;
                    w_des = u(1) ; 
                    v_des = u(2) ;
                    
                    % make the new trajectory
                    Z_new = segway_symbolic_traj(w_0,v_0,w_des,v_des) ;
                    Z_new(1:3,:) = local_to_world(Z(:,end),Z_new(1:3,:)) ;
                    Z = [Z, Z_new(:,2:end)] ;
                    
                    % get the control input
                    U = [U, repmat(u,1,size(Z_new,2)-1)] ;
                end
                
                % create time vector
                t_f = N_U*P.t_edge ;
                T = linspace(0,t_f,size(Z,2)) ;
                
                % convert last bit of the traj to braking
                [T,U,Z] = convert_segway_desired_to_braking_traj(P.t_plan,T,U,Z) ;
                
                if any(isnan(Z(:)))
                    disp('hey!')
                end
            else
                P.vdisp('RRT did not find a new plan',4)
                [T,U,Z] = P.make_plan_for_traj_opt_failure(agent_info) ;
                P.best_traj_indices = [] ;
            end
        end
    end
end