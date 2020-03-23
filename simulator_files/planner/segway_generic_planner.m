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
            P@planner(varargin{:}) ;
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
    end
end