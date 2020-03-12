classdef rover_RTD_planner < generic_RTD_planner
    properties
        point_spacing
        arc_point_spacing
        w_polynomial_info
        current_obstacles_in_FRS_coords
        bounds_as_obstacle
    end
    methods
        %% constructor and setup
        function P = rover_RTD_planner(varargin)
            % set default parameters
            t_plan = 0.5 ;
            t_move = 0.5 ;
            lookahead_distance = 4 ;
            
            % parse arguments
            P@generic_RTD_planner('lookahead_distance',lookahead_distance,...
                't_plan',t_plan,'t_move',t_move,varargin{:})
        end
        
        function load_FRS_files(P,~,~)
            FRS_data = cell(1,1) ;
            FRS_data{1} = load('rover_FRS_deg6_reconstructed_deg6_v0_1.0_to_2.0.mat') ;
            P.FRS = FRS_data ;
        end
        
        function setup(P,agent_info,world_info)
            % call superclass setup
            setup@generic_RTD_planner(P,agent_info,world_info)
            
            % make sure buffer is small enough, i.e. b < \bar{b}
            bbar = agent_info.footprint(2)/2;
            b = P.buffer ;
            
            if b >= bbar
                P.buffer = bbar - 0.001 ;
                P.vdisp('Reducing buffer to be feasible!',2)
            end
            
            % set point spacings
            [r,a] = compute_point_spacings(agent_info.footprint,P.buffer) ;
            P.point_spacing = r ;
            P.arc_point_spacing = a ;
            
            % set FRS buffer
            P.FRS_buffer = P.buffer ;
            
            % set the world bounds with the new buffer
            P.bounds = world_info.bounds + P.FRS_buffer.*[1 -1 1 -1] ;
            
            % create world bounds as an obstacle; note this passes the
            % bounds in as a clockwise polyline, so everything outside of
            % the world bounds counts as inside the polyline if using
            % functions like inpolygon
            xlo = P.bounds(1) ; xhi = P.bounds(2) ;
            ylo = P.bounds(3) ; yhi = P.bounds(4) ;
            
            B = [xlo, xhi, xhi, xlo, xlo ;
                ylo, ylo, yhi, yhi, ylo] ;
            
            P.bounds_as_obstacle = interpolate_polyline_with_spacing(B(:,end:-1:1),P.point_spacing);

        end
        
        %% online planning: process obstacles
        function determine_current_FRS(P,~)
            P.current_FRS_index = 1 ;
            
        end
        
        function process_world_info(P,world_info,~)
            
            F = P.FRS{P.current_FRS_index};
            % process the w polynomial from the rover's FRS; this just
            % extracts the msspoly's powers and coefficients to speed up
            % evaluating the polynomial online
            w_full = F.w - 1.0001 ; % this is so (x,k) \in FRS => w(x,k) > 0
            
            %set second parameter to negative heading (end trajectory plan
            %aligned with road
            psiend = (-P.agent_state(3)-F.psi_end_min)*2/(F.psi_end_max-F.psi_end_min)-1;
            k = F.k ;
            w = subs(w_full,k(2),psiend);
            
            z = F.z ;

            P.w_polynomial_info = decompose_w_polynomial(w,z,k) ;
            
            % get obstacles; for this planner, we assume the obstacles are
            % handed in as a 2-by-N array of boxes separated by columns of
            % NaNs
            
            % extract obstacles
            O = world_info.obstacles ;
            
            
            % buffer and discretize the obstacles
            if ~isempty(O)
                O = buffer_box_obstacles(O,P.FRS_buffer,'a',P.arc_point_spacing) ;
                O = interpolate_polyline_with_spacing(O,P.point_spacing) ;
                
                % move obstacles into FRS coordinates for generating nonlinear
                % constraints with the w polynomial
                x0 = P.F.xoffset(1);
                y0 = P.F.xoffset(2);
                Dx = P.F.xscale(1);
                Dy = P.F.xscale(2);
                
                O_FRS = world_to_FRS(O,P.agent_state(1:3),x0,y0,Dx,Dy) ;
                
                % get rid of points behind the robot, since we don't let the
                % segway go backwards, so those points aren't reachable
                O_log = O_FRS(1,:) >= x0/D ;
                O_FRS = O_FRS(:,O_log) ;
                
                % get rid of obstacle points that are definitely unreachable
                % because they lie outside of the unit circle in the FRS
                % coordinate frame (this is because of how we create the SOS
                % program to compute the FRS)
                O_FRS = crop_points_outside_region(0,0,O_FRS,1) ;
            else
                O_FRS = [] ;
            end
            
            % save buffered, discretized obstacles
            P.current_obstacles = O ;
            
            % save the processed obstacles
            P.current_obstacles_in_FRS_coords = O_FRS ;
        end
        
        %% online planning: cost function
        function create_cost_function(P,agent_info,world_info,start_tic)
            % create waypoint
            P.create_waypoint(agent_info,world_info,start_tic) ;
            
            % get agent state
            z = P.agent_state ;
            
            % rotate waypoint to body-fixed frame
            wp = P.current_waypoint ;
            wp_local = world_to_local(z(1:3),wp(1:2),...
                0,0,1) ;
            
            % create cost function
            w0 = agent_info.state(4,end) ;
            v0 = agent_info.state(5,end) ;
            P.trajopt_problem.cost_function = @(k) P.cost_rover(k,w0,v0,wp_local,start_tic) ;
        end
        
        function [c, gc] = cost_segway(P,k,w0,v0,wp_local,start_tic)
            c = Cfn_segway(w0,v0,wp_local(1),wp_local(2),k(1),k(2)) ;
            gc = dCfn_segway(w0,v0,wp_local(1),wp_local(2),k(1),k(2)) ;
            
            if toc(start_tic) > P.t_plan
                error('Trajopt timed out!')
            end
        end
        
        %% online planning: constraint function
        function create_constraints(P,start_tic)
            % get the processed obstacles
            O = P.current_obstacles_in_FRS_coords ;
            
            if ~isempty(O)
                % remove NaNs
                O_log = isnan(O(1,:)) ;
                O = O(:,~O_log) ;

                % plug in to w polynomial to get list of constraints w(x,k) for
                % each x \in O
                w = P.w_polynomial_info ;
                wk = sub_z_into_w(w,O) ;
                wkcoef = wk.wkcoef ;
                wkpows = wk.wkpows ;

                % create Jacobian of the constraints
                [Jcoef, Jpows] = diff_wk_wrt_k(wk) ;
                N = wk.N ;

                % create constraint function
                P.trajopt_problem.nonlcon_function = @(k) P.nonlcon_segway(k,wkcoef,wkpows,Jcoef,Jpows,N,start_tic) ;
            else
                % if there are no obstacles then we don't need to consider
                % any constraints
                P.trajopt_problem.nonlcon_function = [] ;
            end
            
            % create the inequality constraints and problem bounds
            k = P.FRS{P.current_FRS_index}.k ;
            P.trajopt_problem.Aineq = [] ;
            P.trajopt_problem.bineq = [] ;
            P.trajopt_problem.k_bounds = [-ones(size(k)),ones(size(k))] ;
        end
        
        function [n, neq, gn, gneq] = nonlcon_segway(P,k,wkcoef,wkpows,Jcoef,Jpows,N,start_tic)
            % constraint is active when p(k) > 0
            n = eval_w(k,wkcoef,wkpows) ;
            neq = zeros(size(n)) ;
            gn = eval_J(k,Jcoef,Jpows,N)' ;
            gneq = zeros(size(gn)) ;
            
            if toc(start_tic) > P.t_plan
                error('Trajopt timed out!')
            end
        end
        
        %% online planning: create output given successful replan
        function [T_out,U_out,Z_out] = create_desired_trajectory(P,~,k_opt)
            % get desired speed and yaw rate
            v_max = P.FRS{P.current_FRS_index}.vmax ;
            v_des = (v_max/2)*k_opt(2) + (v_max/2) ;
            
            w_max = P.FRS{P.current_FRS_index}.wmax ;
            w_des = w_max*k_opt(1) ;
            
            % set up timing
            t_f = P.FRS{P.current_FRS_index}.T ;
            t_sample = 0.01 ;
            t_plan = P.t_move ;
            t_stop = t_f - t_plan ;
            T_out = unique([0:t_sample:t_f,t_f]);
            
            % constant velocity until t_plan (note t_plan = t_move for real
            % time planning)
            phase_1 = @(t,z) [v_des*cos(z(3));v_des*sin(z(3));w_des];
            
            % deceleration
            phase_2 = @(t,z) (1-(t-t_plan)/t_stop)*[v_des*cos(z(3));v_des*sin(z(3));w_des];
            
            % compute dubins path trajectories
            [T_1,X_1] = ode45(@(t,z)phase_1(t,z),[0,t_plan],[0;0;0]);
            [T_2,X_2] = ode45(@(t,z)phase_2(t,z),[t_plan,t_f],X_1(end,:));
            Z_out = NaN(5,length(T_1) + length(T_2)) ;
            Z_out(1:3,:) = [X_1', X_2(2:end,:)', X_2(end,:)'] ;
            
            % create speed and yaw rate trajectories
            v_traj = [repmat(v_des,[1,length(T_1)]),(1-(T_2(2:end-1)'-t_plan)/t_stop)*v_des,0,0];
            w_traj = [repmat(w_des,[1,length(T_1)]),(1-(T_2(2:end-1)'-t_plan)/t_stop)*w_des,0,0];
            
            % make full state trajectories
            Z_out(4:5,:) = [w_traj;v_traj];
            
            % make sure the output trajectory timing is right
            if T_2(end) ~= t_f
                Z_out = interp1([T_1;T_2(2:end);t_f],Z_out',T_out)';
            else
                Z_out = interp1([T_1;T_2(2:end)],Z_out(:,1:end-1)',T_out)';
            end
            
            % the nominal input is the desired speed and yaw rate
            U_out = Z_out([4,5],:);
        end
        
        %% plotting
        function plot(P,~)
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on ;
            end
            
            % plot current obstacles
            O = P.current_obstacles ;
            
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
            
            % plot current waypoint
            wp = P.current_waypoint ;
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
            
            if hold_check
                hold off
            end
        end
        
        function plot_at_time(P,~)
            P.vdisp('Shreyas write this!',10)
        end
    end
end
